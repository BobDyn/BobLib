"""Sampled steering retains its recurrence and prevents integral windup."""
import csv
import shutil
import subprocess
from pathlib import Path

import pytest


def test_sampled_steering(tmp_path):
    omc = shutil.which('omc')
    if omc is None:
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    (tmp_path / 'Sampled.mo').write_text('''model Sampled
  extends BobLibTest.Drivers.TestStandardDriver(driver(useMode=3,
    steerStart=0, targetAy=1, steadyStateAyRampRate=2,
    steadyStateAyGain=0.025, steadyStateAyTi=1,
    linearitySlopeSamplePeriod=0.1));
end Sampled;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{root}/Tests/BobLibTest/package.mo");
loadFile("Sampled.mo");
simulate(Sampled, stopTime=1, numberOfIntervals=100, tolerance=1e-10,
  outputFormat="csv", variableFilter="driver.steadyState(AyIntegral|SteerCmd)");
getErrorString();
''')
    result = subprocess.run([omc, 'run.mos'], cwd=tmp_path, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
    assert 'The simulation finished successfully' in result.stdout, result.stdout
    with (tmp_path / 'Sampled_res.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    integral = command = 0.0
    for n in range(11):
        t = n / 10
        xi = min(1, t / 0.5)
        error = 3 * xi**2 - 2 * xi**3
        integral += error * 0.1
        command += (0.1 / 0.15) * (0.025 * (error + integral) - command)
        # The last row at an event contains its post-event value.
        row = [r for r in rows if abs(float(r['time']) - t) < 1e-9][-1]
        assert float(row['driver.steadyStateAyIntegral']) == pytest.approx(integral, abs=1e-10)
        assert float(row['driver.steadyStateSteerCmd']) == pytest.approx(command, abs=1e-7)



@pytest.mark.parametrize('direction', [-1, 1])
def test_steering_saturation_does_not_wind_up(tmp_path, direction):
    omc = shutil.which('omc')
    if omc is None:
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    (tmp_path/'Saturated.mo').write_text(f'''model Saturated
  extends BobLibTest.Drivers.TestStandardDriver(driver(useMode=3,
    steerStart=0, targetAy={direction}, steadyStateAyRampRate=2,
    steadyStateMaxHandwheel=0.01));
equation
  assert(abs(driver.steadyStateSteerCmd) <= 0.010001, "Steering limit exceeded");
  assert(abs(driver.steadyStateAyIntegral) < 0.4,
    "Unreachable lateral target must not wind up the steering integral");
end Saturated;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{root}/Tests/BobLibTest/package.mo");
loadFile("Saturated.mo");
simulate(Saturated,stopTime=5);
getErrorString();
''')
    result = subprocess.run([omc, 'run.mos'], cwd=tmp_path, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
    assert 'The simulation finished successfully' in result.stdout, result.stdout


@pytest.mark.parametrize('start,target,load', [(0.2, 1, 100), (0.2, -1, 100), (-1, 1, 1000)])
def test_ramp_stop_waveform(tmp_path, start, target, load):
    omc = shutil.which('omc')
    if omc is None:
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    (tmp_path / 'Ramp.mo').write_text(f'''model Ramp
  extends BobLibTest.Drivers.TestStandardDriver(driver(useMode=0,
    steerStart={start}, targetAy={target}), normalLoad_1(k={load}));
end Ramp;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{root}/Tests/BobLibTest/package.mo");
loadFile("Ramp.mo");
simulate(Ramp, stopTime=1, numberOfIntervals=100, tolerance=1e-10,
  outputFormat="csv", variableFilter="driver.(frSteerCmd|handwheelRateCmd)");
getErrorString();
''')
    result = subprocess.run([omc, 'run.mos'], cwd=tmp_path, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
    assert 'The simulation finished successfully' in result.stdout, result.stdout
    with (tmp_path / 'Ramp_res.csv').open() as stream:
        for row in csv.DictReader(stream):
            t = float(row['time'])
            elapsed = max(0, t - max(0, start))
            if load < 200:
                u = min(elapsed / 0.18, 1)
                angle = 0.14 * 0.18 * (u - u**3 + 0.5 * u**4)
                rate = 0.14 * (1 - 3 * u**2 + 2 * u**3) if t >= start else 0
            else:
                angle, rate = 0.14 * elapsed, 0.14
            assert float(row['driver.frSteerCmd']) == pytest.approx(target * angle, abs=1e-8)
            if abs(t - start) > 1e-8:
                assert float(row['driver.handwheelRateCmd']) == pytest.approx(target * rate, abs=1e-8)


def test_invalid_driver_calibration_has_actionable_error(tmp_path):
    omc = shutil.which('omc')
    if omc is None:
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    (tmp_path/'InvalidDriver.mo').write_text('''model InvalidDriver
  extends BobLibTest.Drivers.TestStandardDriver(driver(useMode=3, steadyStateAyTi=0));
end InvalidDriver;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{root}/Tests/BobLibTest/package.mo");
loadFile("InvalidDriver.mo");
simulate(InvalidDriver,stopTime=0.1);
getErrorString();
''')
    result = subprocess.run([omc, 'run.mos'], cwd=tmp_path, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
    assert 'BOBLIB_INVALID_DRIVER_CONFIG' in result.stdout
    assert 'The simulation finished successfully' not in result.stdout
