"""An actual CS solver must honor the requested FMI integration tolerance."""
import importlib.util
import os
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to build the compiler reproducer')
def test_cs_setup_tolerance(tmp_path, monkeypatch):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    from fmpy.fmi1 import FMICallException

    spec = importlib.util.spec_from_file_location('tolerance_export', ROOT/'Tools/export_fmu.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    monkeypatch.setattr(exporter, 'parameter_manifest', lambda *args, **kwargs: {})
    patch = exporter.apply_cs_tolerance
    source = (ROOT/'development/omc_repros/CoSimulationTolerance.mo').read_text()
    for corrected in (False, True):
        monkeypatch.setattr(exporter, 'apply_cs_tolerance', patch if corrected else lambda text: text)
        artifact = exporter.build_source('CoSimulationTolerance', source,
            tmp_path/str(corrected), 2, {'kind': 'compiler-regression'}, controller=True)
        errors = {}
        for tolerance in (.01, 1e-9):
            result = fmpy.simulate_fmu(str(artifact), stop_time=10, output_interval=.1,
                relative_tolerance=tolerance, output=['solution'])
            errors[tolerance] = np.max(np.abs(result['solution'] - np.exp(-result['time'])))
        if not corrected:
            assert errors[.01] == errors[1e-9]
            continue
        assert errors[1e-9] < 1e-7
        assert errors[.01] > 100 * errors[1e-9]
        # None means toleranceDefined=false; the runtime must retain its default.
        for tolerance in (None, 0., -1., float('nan'), float('inf')):
            directory = fmpy.extract(str(artifact))
            instance = fmpy.instantiate_fmu(directory, fmpy.read_model_description(str(artifact)),
                                             fmi_type='CoSimulation')
            try:
                if tolerance is None:
                    instance.setupExperiment()
                    instance.enterInitializationMode()
                    instance.exitInitializationMode()
                    instance.doStep(currentCommunicationPoint=0, communicationStepSize=.1)
                    vr = next(v.valueReference for v in fmpy.read_model_description(str(artifact)).modelVariables
                              if v.name == 'solution')
                    assert abs(instance.getReal([vr])[0] - np.exp(-.1)) < 1e-5
                    instance.terminate()
                else:
                    with pytest.raises(FMICallException):
                        instance.setupExperiment(tolerance=tolerance)
            finally:
                instance.freeInstance()
                shutil.rmtree(directory)
