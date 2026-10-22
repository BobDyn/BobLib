"""Invalid exported calibration must fail with an actionable component diagnostic."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('fixture,modification,diagnostic', [
    ('ElectricDrives.Internal.TestPowerLimitedMotor',
     'motoringForward(pMotor(eta_reg=0))', 'PowerLimitedMotor: efficiencies'),
    ('ElectricDrives.Internal.TestPowerLimitedMotor',
     'motoringForward(pMotor(tau_tau=-1))', 'PowerLimitedMotor: w_eps and tau_tau'),
    ('PowerElectronics.TestInverterDC',
     'dut(pInverter(powerFractionTable={0,0.15,0.05,0.35,0.65,1}))',
     'InverterDC: powerFractionTable breakpoints'),
])
def test_invalid_calibration_fails(tmp_path, fixture, modification, diagnostic):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path / 'Invalid.mo').write_text(
        f'model Invalid\n extends BobLibTest.{fixture}({modification});\nend Invalid;\n')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("Invalid.mo");
simulate(Invalid,stopTime=0.01);getErrorString();
''')
    result = subprocess.run(['omc', 'run.mos'], cwd=tmp_path, capture_output=True,
                            text=True, timeout=180)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert diagnostic in output, output
    assert 'The simulation finished successfully' not in output, output
    assert 'assert' in output.lower(), output
