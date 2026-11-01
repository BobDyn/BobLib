"""The explicit brake port preserves the existing upstream shaft station."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_differential_brake_station_and_vehicle_contract(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "BrakePort.mo"
    fixture.write_text('''package BrakePort
  model Legacy
    extends BobLibTest.Drivelines.TestRearFinalDriveDifferential;
    Modelica.Mechanics.Rotational.Sources.ConstantTorque brake(tau_constant=-20);
  equation
    connect(brake.flange, dut.transmissionFlange.flange);
  end Legacy;
  model ExplicitPort
    extends BobLibTest.Drivelines.TestRearFinalDriveDifferential(dut(useDifferentialInputBrake=true));
    Modelica.Mechanics.Rotational.Sources.ConstantTorque brake(tau_constant=-20);
  equation
    connect(brake.flange, dut.differentialBrakeShaft.flange);
    assert(abs(brake.flange.phi-dut.transmissionFlange.flange.phi)<1e-10,
      "Brake shaft must be upstream of final reduction");
  end ExplicitPort;
  model Pair
    parameter BobLib.Drivelines.Types.DifferentialArchitecture architecture;
    Legacy legacy(
      dut(pDriveline(inputRatio=3), differentialArchitecture=architecture));
    ExplicitPort explicitPort(
      dut(pDriveline(inputRatio=3), differentialArchitecture=architecture));
  equation
    assert(abs(legacy.left.w-explicitPort.left.w)<1e-7, "Left wheel behavior changed");
    assert(abs(legacy.right.w-explicitPort.right.w)<1e-7, "Right wheel behavior changed");
    assert(abs(legacy.dut.leftHalfshaftTorque-explicitPort.dut.leftHalfshaftTorque)<1e-6,
      "Brake port moved across reduction or compliance");
  end Pair;
''' + "".join(f'''  model {architecture}
    extends Pair(architecture=BobLib.Drivelines.Types.DifferentialArchitecture.{architecture});
  end {architecture};
''' for architecture in ("Open", "ClutchLSD", "TorqueBiasing", "Locked")) + '''
  model Vehicle
    extends BobLib.Experiments.Standards.VehicleFMI(
      redeclare BobLib.DriverEnvironments.CommandedDriveByWire driverEnvironment,
      rearBrakePlacement=BobLib.Brakes.Types.Placement.DifferentialInput);
  end Vehicle;
  model VehicleOutboard
    extends Vehicle(rearBrakePlacement=BobLib.Brakes.Types.Placement.Outboard);
  end VehicleOutboard;
  model VehicleInboard
    extends Vehicle(rearBrakePlacement=BobLib.Brakes.Types.Placement.Inboard);
  end VehicleInboard;
end BrakePort;
''')
    script = tmp_path / "run.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
checkModel(BrakePort.Vehicle);
getErrorString();
checkModel(BrakePort.VehicleOutboard);
getErrorString();
checkModel(BrakePort.VehicleInboard);
getErrorString();
''' + "".join(f'''simulate(BrakePort.{architecture}, stopTime=0.1, tolerance=1e-8);
getErrorString();
''' for architecture in ("Open", "ClutchLSD", "TorqueBiasing", "Locked")))
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=600, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    counts = re.findall(r"(\d+) equation\(s\) and (\d+) variable\(s\)", output)
    assert len(counts) == 3 and all(equations == variables for equations, variables in counts), output
    assert output.count("The simulation finished successfully") == 4, output
