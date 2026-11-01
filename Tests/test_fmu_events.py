"""CS must localize state events and retain scheduled events in the same step."""
import importlib.util
import os
import shutil
from pathlib import Path

import pytest


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to build the compiler reproducer')
def test_cs_event_localization(tmp_path, monkeypatch):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('event_export', root/'Tools/export_fmu.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    monkeypatch.setattr(exporter, 'parameter_manifest', lambda *args, **kwargs: {})
    patch = exporter.localize_cs_events
    source = '''model EventLocalization
  parameter Real period=0.125 annotation(Evaluate=false);
  output Real x(start=0, fixed=true);
  output Integer crossings(start=0, fixed=true);
  output Integer ticks(start=0, fixed=true);
  output Real lastCrossing(start=0, fixed=true);
equation
  der(x)=1;
  when x >= period then
    reinit(x,0);
    crossings=pre(crossings)+1;
    lastCrossing=time;
  end when;
  when sample(0.1,0.1) then
    ticks=pre(ticks)+1;
  end when;
end EventLocalization;
'''
    for corrected in (False, True):
        monkeypatch.setattr(exporter, 'localize_cs_events', patch if corrected else lambda a,b:(a,b))
        archive = exporter.build_source('EventLocalization', source, tmp_path/str(corrected),
            2, {'kind':'compiler-regression'}, controller=True)
        result = fmpy.simulate_fmu(str(archive), stop_time=1.05, output_interval=.05,
            relative_tolerance=1e-9, output=['x','crossings','ticks','lastCrossing'])
        final = result[-1]
        if corrected:
            import zipfile
            with zipfile.ZipFile(archive) as contents, pytest.raises(ValueError, match='Unsupported OMC CS source'):
                patch(contents.read('sources/simulation/solver/cvode_solver.c').decode(),
                      contents.read('sources/fmi-export/fmu2_model_interface.c').decode())
            with pytest.raises(ValueError, match='Unsupported OMC CS source'):
                patch('changed compiler source', 'changed compiler source')
        if not corrected:
            assert abs(final['x']-.05) > .01 or final['crossings'] != 8
            continue
        assert final['crossings'] == 8
        assert final['ticks'] == 10
        assert abs(final['lastCrossing']-1) < 1e-7
        assert abs(final['x']-.05) < 1e-7

        # Several state crossings must be handled within one communication step.
        dense = fmpy.simulate_fmu(str(archive), stop_time=1.05, output_interval=.35,
            relative_tolerance=1e-9, start_values={'period':.03125},
            output=['x','crossings','ticks','lastCrossing'])[-1]
        assert dense['crossings'] == 33
        assert dense['ticks'] == 10
        assert abs(dense['lastCrossing']-1.03125) < 1e-7
        assert abs(dense['x']-.01875) < 1e-7
