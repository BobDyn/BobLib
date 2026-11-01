"""Actual OMC/FMI regression for a nonlinear system with an already-valid guess."""
import hashlib
import importlib.util
import os
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to build the compiler reproducer')
def test_homotopy_early_success_preserves_solution(tmp_path, monkeypatch):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    from fmpy.fmi1 import FMICallException

    spec = importlib.util.spec_from_file_location('homotopy_export', ROOT/'Tools/export_fmu.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    # This compiler reproducer has neither a vehicle record nor driver ports.
    monkeypatch.setattr(exporter, 'parameter_manifest', lambda *args, **kwargs: {})
    source = (ROOT/'development/omc_repros/HomotopyEarlySuccess.mo').read_text()
    patch = exporter.guard_homotopy_initial_guess
    for corrected in (False, True):
        monkeypatch.setattr(exporter, 'guard_homotopy_initial_guess', patch if corrected else lambda text: text)
        artifact = exporter.build_source('HomotopyEarlySuccess', source,
            tmp_path/('corrected' if corrected else 'original'), 2,
            {'kind': 'compiler-regression', 'corrected': corrected,
             'test_source_sha256': hashlib.sha256(source.encode()).hexdigest()}, controller=True)
        if not corrected:
            with pytest.raises(FMICallException, match='fmi2ExitInitializationMode'):
                fmpy.simulate_fmu(str(artifact), stop_time=.5, output_interval=.002,
                                  output=['gap', 'normalZ'])
        else:
            result = fmpy.simulate_fmu(str(artifact), stop_time=.5, output_interval=.002,
                                       output=['gap', 'normalZ'])
            np.testing.assert_allclose(result['gap'], -.01, rtol=0, atol=1e-8)
            np.testing.assert_allclose(result['normalZ'], 1, rtol=0, atol=1e-8)


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to build the driver FMU')
def test_driver_export_without_nonlinear_solver(tmp_path):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    import json

    spec = importlib.util.spec_from_file_location('driver_export', ROOT/'Tools/export_fmu.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    artifact = exporter.build_driver(tmp_path, 2)
    manifest = json.loads((artifact.parent/'manifest.json').read_text())
    assert 'homotopy_initial_guess_return' not in manifest['workarounds']
    names = ['time', 'speed', 'ay'] + [f'loads[{i}]' for i in range(1, 5)]
    inputs = np.array([(0., 15., 0., 1000., 1000., 1000., 1000.),
                       (.1, 15., 0., 1000., 1000., 1000., 1000.)],
                      dtype=[(name, float) for name in names])
    result = fmpy.simulate_fmu(str(artifact), stop_time=.1, input=inputs,
                               output_interval=.002, output=['accelerator', 'brake'])
    np.testing.assert_allclose(result['accelerator'], 0, rtol=0, atol=1e-12)
    np.testing.assert_allclose(result['brake'], 0, rtol=0, atol=1e-12)
    # Exercise the actual initialized limits; editable metadata alone is insufficient.
    for values, drive_limit, brake_limit in [
        ({}, 240., 1500.),
        ({'tau_max': 220., 'finalDriveRatio': 3.31, 'mechanicalBrakeTorqueLimit': 900.}, 728.2, 900.),
        ({}, 240., 1500.),
    ]:
        result = fmpy.simulate_fmu(str(artifact), stop_time=.1, input=inputs,
            start_values=values, output_interval=.1,
            output=['driver.driveTorqueCapacity', 'driver.brakeTorqueCapacity'])
        np.testing.assert_allclose(result['driver.driveTorqueCapacity'], drive_limit, rtol=0, atol=1e-9)
        np.testing.assert_allclose(result['driver.brakeTorqueCapacity'], brake_limit, rtol=0, atol=1e-9)
