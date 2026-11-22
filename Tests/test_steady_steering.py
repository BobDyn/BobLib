"""Continuous steering stays rate bounded across sampled controller updates."""
import csv
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_continuous_steering_and_vehicle_balance(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path / 'Steering.mo').write_text('''model Steering
  extends BobLibTest.Drivers.TestStandardDriver(driver(
    useMode=3, steerStart=0.05, targetAy=8, steadyStateAyGain=0.2,
    steadyStateMaxHandwheel=0.3, steadyStateMaxHandwheelRate=0.4));
  Real handwheelRate = der(driver.steadyStateSteerCmd);
equation
  assert(abs(der(driver.steadyStateSteerCmd)) <= 0.400001,
    "Continuous actuator must respect its rate limit");
  assert(abs(driver.steadyStateSteerCmd) <= 0.300001,
    "Continuous actuator must respect its angle limit");
end Steering;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("Steering.mo");
checkModel(BobLib.Experiments.Standards.VehicleSim);
getErrorString();
simulate(Steering, stopTime=2, numberOfIntervals=2000, tolerance=1e-8, outputFormat="csv");
getErrorString();
''')
    result = subprocess.run(['omc', '--numProcs=1', 'run.mos'], cwd=tmp_path,
                            text=True, capture_output=True, timeout=180)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert result.returncode == 0 and 'Error:' not in output, output
    assert 'The simulation finished successfully' in output, output
    counts = re.findall(r'([0-9]+) equation\(s\) and ([0-9]+) variable\(s\)', output)
    assert counts and all(a == b for a, b in counts), output
    with (tmp_path / 'Steering_res.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    for a, b in zip(rows, rows[1:], strict=False):
        dt = float(b['time']) - float(a['time'])
        change = abs(float(b['driver.steadyStateSteerCmd']) - float(a['driver.steadyStateSteerCmd']))
        assert change <= 0.4*dt + 1e-7
        rate_change = abs(float(b['handwheelRate']) - float(a['handwheelRate']))
        assert rate_change <= (2*0.4/0.15)*dt + 1e-6
    assert float(rows[-1]['driver.steadyStateSteerCmd']) > 0.29
