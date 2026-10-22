"""Behavioral execution of every concrete fixture in the mirrored coverage map."""
import shutil
import subprocess
from pathlib import Path

import pytest
from modelica_test_library import coverage, run_case


@pytest.mark.parametrize('case', [row for row in coverage() if row['mode'] == 'simulate'] + [{
    'model': 'BobLibTest.Chassis.ChassisBusContractSmoke',
    'test': 'BobLibTest.Chassis.ChassisBusContractSmoke',
    'scope': 'integration', 'mode': 'simulate', 'stop_time': 0.1,
}, {
    'model': 'BobLibTest.Chassis.Suspension.Tires.TestFourMF52Transient',
    'test': 'BobLibTest.Chassis.Suspension.Tires.TestFourMF52Transient',
    'scope': 'integration', 'mode': 'simulate', 'stop_time': 2.0,
}] + [{
    'model': f'BobLibTest.Chassis.Suspension.Components.TestBellcrank{pickups}Motion',
    'test': f'BobLibTest.Chassis.Suspension.Components.TestBellcrank{pickups}Motion',
    'scope': 'component', 'mode': 'simulate', 'stop_time': 1.0,
} for pickups in (2, 3)] + [{
    'model': f'BobLibTest.Chassis.Suspension.Components.Test{component}Preload',
    'test': f'BobLibTest.Chassis.Suspension.Components.Test{component}Preload',
    'scope': 'component', 'mode': 'simulate', 'stop_time': 0.01,
} for component in ('ForceOnlyRod', 'ShockLinkage')],
                         ids=lambda row: row['model'])
def test_modelica_component_simulates(case, tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica omc is not installed')
    result = run_case(case, tmp_path, timeout=600)
    log = tmp_path / case['test'] / 'omc.log'
    assert result['status'] == 'passed', log.read_text()


def test_bilinear_grid_validation(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica omc is not installed')
    root = Path(__file__).resolve().parents[1]
    models = tmp_path / 'GridValidation.mo'
    models.write_text("""package GridValidation
      model ShortGrid
        Real z = BobLib.Aero.Internal.Bilinear2D(time, 2, {0}, {1,3}, [10,30]);
      end ShortGrid;
      model DuplicateGrid
        Real z = BobLib.Aero.Internal.Bilinear2D(time, 2, {0,0}, {1,3}, [10,30;20,40]);
      end DuplicateGrid;
      model ReversedGrid
        Real z = BobLib.Aero.Internal.Bilinear2D(time, 2, {0,1}, {3,1}, [10,30;20,40]);
      end ReversedGrid;
    end GridValidation;
""")
    script = tmp_path / 'grid.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{models}");
BobLib.Aero.Internal.Bilinear2D(0.5, 2, {{0,1}}, {{1,3}}, [10,30;20,40]);
getErrorString();
simulate(GridValidation.ShortGrid, stopTime=0.01);getErrorString();
simulate(GridValidation.DuplicateGrid, stopTime=0.01);getErrorString();
simulate(GridValidation.ReversedGrid, stopTime=0.01);getErrorString();
''')
    result = subprocess.run(['omc', str(script)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=60, check=True)
    (tmp_path / 'omc.log').write_text(result.stdout + result.stderr)
    assert '\n25.0\n' in result.stdout, result.stdout
    assert 'The simulation finished successfully' not in result.stdout, result.stdout
    for message in ('active counts must be between two', 'xGrid must be strictly increasing',
                    'yGrid must be strictly increasing'):
        assert message in result.stdout, result.stdout
