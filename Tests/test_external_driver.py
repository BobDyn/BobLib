"""Stock VI driver interoperates with BobLib's scalar-command actuator path."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_normalized_driver_limits_and_feedback(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    script = tmp_path / 'run.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
''' + ''.join(f'''simulate(BobLibTest.DriverEnvironments.TestExternalDriver,
stopTime=0.1,simflags="-override=steeringNormalized={steering},pedal={pedal}");
getErrorString();
''' for steering,pedal in [(0,0),(.5,.4),(1,1)]))
    result = subprocess.run(['omc',str(script)],cwd=tmp_path,text=True,
                            capture_output=True,timeout=180,check=True)
    output=result.stdout+result.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert output.count('The simulation finished successfully')==3, output
