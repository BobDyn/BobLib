"""Vehicle envelope exits retain results and distinguish limits from convergence."""
import csv
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def vehicle_executable(tmp_path_factory):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is required")
    work = tmp_path_factory.mktemp("vehicle-termination")
    (work / "build.mos").write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
setCommandLineOptions("--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none --homotopyApproach=equidistantLocal");
buildModel(BobLib.Experiments.Standards.VehicleSim, stopTime=1,
  numberOfIntervals=1000, tolerance=1e-6, outputFormat="csv",
  fileNamePrefix="vehicle", variableFilter="time|terminationStatus|Fz_.*|accY|yawVel");
getErrorString();
''')
    result = subprocess.run(["omc", "build.mos"], cwd=work, capture_output=True,
                            text=True, timeout=600, check=True)
    (work / "build.log").write_text(result.stdout + result.stderr)
    assert "Error:" not in result.stdout, result.stdout
    assert (work / "vehicle").is_file(), result.stdout
    return work


CASES = [
    ("running", "driver.steerStart=20", 0, None),
    ("settled", "driver.useMode=2,driver.steerStart=0.01,driver.frRampSteerHeight=0", 1,
     "Reached open-loop step-steer steady-state"),
    ("tire", "driver.steerStart=0.01,tireLiftTerminateLoad=1000000", 4,
     "BOBLIB_TIRE_LOAD_LIMIT"),
    ("sideslip", "driver.steerStart=0.01,sideslipTerminate=0.000000001,spinoutHoldDuration=0.02", 5,
     "BOBLIB_SIDESLIP_LIMIT"),
    ("ramp_timeout", "driver.steerStart=0.01,driver.tireNormalLoadMin=10000,"
     "driver.handwheelRampStopDuration=0.01,settleTimeout=0.02,der_yawVelTol=0,handwheelRateTol=0", 3,
     "BOBLIB_SETTLING_TIMEOUT"),
    ("closed_timeout", "driver.useMode=3,driver.steerStart=0.01,driver.targetAy=1,"
     "driver.steadyStateAyRampRate=1000,steadyStateHoldDuration=0.01,"
     "steadyStateSettleTimeout=0.05,steadyStateAyTolerance=0.000000000001", 3,
     "BOBLIB_SETTLING_TIMEOUT"),
]


@pytest.mark.parametrize("name,overrides,status,message", CASES)
def test_vehicle_exit_status(vehicle_executable, name, overrides, status, message):
    work = vehicle_executable
    result = subprocess.run([str(work / "vehicle"), f"-override={overrides}",
                             f"-r={name}.csv", "-stopTime=1"], cwd=work, capture_output=True,
                            text=True, timeout=120)
    output = result.stdout + result.stderr
    (work / f"{name}.log").write_text(output)
    assert "override variable name not found" not in output, output
    assert result.returncode == 0, output
    assert "The simulation finished successfully" in output, output
    if message:
        assert message in output, output
    with (work / f"{name}.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert rows, output
    assert int(rows[-1]["terminationStatus"]) == status, (rows[-1], output)
    if message:
        assert float(rows[-1]["time"]) < 1, output


def test_invalid_monitor_configuration_still_fails(vehicle_executable):
    work = vehicle_executable
    result = subprocess.run([str(work / "vehicle"), "-override=tireLiftTerminateLoad=-1",
                             "-r=invalid.csv"], cwd=work, capture_output=True,
                            text=True, timeout=120)
    output = result.stdout + result.stderr
    (work / "invalid.log").write_text(output)
    assert result.returncode != 0, output
    assert "BOBLIB_INVALID_MONITOR_CONFIG" in output, output
