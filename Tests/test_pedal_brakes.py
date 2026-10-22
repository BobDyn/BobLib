"""Physical pedal travel and force use the existing caliper reaction paths."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("travel,rate", [(0,0), (0.025,0), (0.05,0), (0.06,0), (-0.01,0), (0.01,0.1)])
def test_physical_pedal(tmp_path, travel, rate):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
getErrorString();
loadString("model PedalCase extends BobLibTest.Brakes.TestPedalBrakes(travel={travel},travelRate={rate}); end PedalCase;");
simulate(PedalCase,stopTime=0.1);
getErrorString();
''')
    r=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,capture_output=True,check=True,timeout=120)
    output=r.stdout+r.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert 'The simulation finished successfully' in output,output
