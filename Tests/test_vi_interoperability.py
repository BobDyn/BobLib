"""Run stock VI replacements and check the explicit EV measurement extensions."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_vi_replacements_and_measurement_adapters(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "MeasuredVI.mo"
    fixture.write_text('''package MeasuredVI
  model MountedMotor
    extends VehicleInterfaces.ElectricDrives.SimpleMotorDC;
    parameter BobLib.Records.VehicleRecord.ElectricDrives.MotorRecord pMotor;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a chassisFrame if usingMultiBodySystem;
    VehicleInterfaces.PowertrainMounts.SingleSystemRigidMount mount(
      r_ChassisToMount1=pMotor.mountOffset) if usingMultiBodySystem;
  equation
    connect(chassisFrame,mount.chassisFrame);
    connect(mount.mount_1,shaft_b.bearingFrame);
  end MountedMotor;
  model Motor
    extends BobLib.Experiments.Standards.VehicleFMI(
      redeclare MountedMotor motor(pMotor=pVehicle.motor));
    Modelica.Blocks.Sources.RealExpression speed(y=der(motor.shaft_b.flange.phi));
  equation
    connect(speed.y, controlBus.electricMotorBus.speed);
  end Motor;
  model Battery
    extends BobLib.Experiments.Standards.VehicleFMI(
      redeclare VehicleInterfaces.EnergyStorage.Battery battery(V=400));
    Modelica.Blocks.Sources.RealExpression voltage(y=battery.pin_p.v-battery.pin_n.v);
    Modelica.Blocks.Sources.RealExpression current(y=-battery.pin_p.i);
  equation
    connect(voltage.y, controlBus.batteryBus.voltage);
    connect(current.y, controlBus.batteryBus.current);
  end Battery;
end MeasuredVI;
''')
    simulations = [
        "BobLibTest.Experiments.Standards.BaseClasses.TestVIInteroperability",
        "BobLibTest.Transmissions.TestFixedRatioTransmission",
        "BobLibTest.DriverEnvironments.TestAutomaticDriveByWire",
    ]
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
checkModel(MeasuredVI.Motor);
getErrorString();
checkModel(MeasuredVI.Battery);
getErrorString();
''' + "".join(f"simulate({model}, stopTime=0.1);\ngetErrorString();\n"
              for model in simulations))
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    counts = re.findall(r"(\d+) equation\(s\) and (\d+) variable\(s\)", output)
    assert len(counts) == 2 and all(a == b for a, b in counts), output
    assert output.count("The simulation finished successfully") == len(simulations), output
