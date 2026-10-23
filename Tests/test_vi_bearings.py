"""VI bearing frames carry upright loads without inheriting wheel spin."""
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_bearing_load_path_and_vehicle_flags(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "Bearings.mo"
    fixture.write_text('''package Bearings
  model Wheel
    inner parameter Boolean headless=true;
    inner parameter Real linkDiameter=0.02;
    inner Modelica.Mechanics.MultiBody.World world(g=0, enableAnimation=false);
    Modelica.Mechanics.MultiBody.Parts.Fixed upright;
    BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheel(pWheel(rotation(wheelJ=0.1),geometry(R0=0.2, rimR0=0.125, rimWidth=0.18, staticAlpha=10, staticGamma=15)),exposeWheelBearing=true);
    Modelica.Mechanics.Rotational.Sources.Speed spin(exact=true, w_ref=3);
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque load(
      animation=false, force={1,2,3}, torque={4,5,6},
      resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world);
  equation
    connect(upright.frame_b, wheel.chassisFrame);
    connect(spin.flange, wheel.hubFlange);
    connect(load.frame_b, wheel.bearingFrame);
    assert(sum(Modelica.Mechanics.MultiBody.Frames.angularVelocity1(wheel.bearingFrame.R).^2)<1e-12,
      "Bearing frame must not spin with the rotor");
    assert(abs(der(wheel.hubFlange.phi)-3)<1e-10, "Wheel must spin independently of bearing");
    assert(sum((Modelica.Mechanics.MultiBody.Frames.resolve1(wheel.chassisFrame.R, wheel.chassisFrame.f)+{1,2,3}).^2)<1e-12,
      "Bearing force must reach the upright exactly once");
    assert(sum((Modelica.Mechanics.MultiBody.Frames.resolve1(wheel.chassisFrame.R, wheel.chassisFrame.t)+{4,5,6}).^2)<1e-12,
      "Bearing torque must reach the upright exactly once");
  end Wheel;
  model MovingWheel
    inner parameter Boolean headless=true;
    inner parameter Real linkDiameter=0.02;
    inner Modelica.Mechanics.MultiBody.World world(g=0, enableAnimation=false, driveTrainMechanics3D=true);
    Modelica.Mechanics.MultiBody.Joints.Revolute carrier(n={0,1,0},useAxisFlange=true);
    Modelica.Mechanics.Rotational.Sources.Speed carrierSpeed(exact=true,w_ref=2);
    Modelica.Mechanics.Rotational.Sources.Speed spin(exact=true,w_ref=3);
    BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheel(pWheel(rotation(wheelJ=0.1),geometry(R0=0.2,rimR0=0.125,rimWidth=0.18,staticAlpha=0,staticGamma=0)),includeRotorEffects=true,exposeWheelBearing=true);
  equation
    connect(world.frame_b,carrier.frame_a);
    connect(carrierSpeed.flange,carrier.axis);
    connect(carrier.frame_b,wheel.chassisFrame);
    connect(spin.flange,wheel.hubFlange);
    assert(abs(wheel.absoluteSpinSpeed-5)<1e-9,
      "Tire spin must include the carrier angular velocity");
  end MovingWheel;
''' + "".join(f'''  model Chassis{i}
    extends BobLib.Chassis.DoubleWishbone(includeWheelBearings={str(bearing).lower()});
  end Chassis{i};
  model Vehicle{i}
    extends BobLib.Experiments.Standards.VehicleFMI(
      includeRotorEffects={str(driveline).lower()},
      redeclare Chassis{i} chassis(pChassis=pVehicle.chassis));
  end Vehicle{i};
''' for i, (bearing, driveline) in enumerate([(False, False), (True, False), (False, True), (True, True)])) + 'end Bearings;\n')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
''' + "".join(f"checkModel(Bearings.Vehicle{i});\ngetErrorString();\n" for i in range(4)) +
                      "simulate(Bearings.Wheel, stopTime=0.1);\ngetErrorString();\n" +
                      "simulate(Bearings.MovingWheel, stopTime=0.1);\ngetErrorString();\n")
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    counts = re.findall(r"(\d+) equation\(s\) and (\d+) variable\(s\)", output)
    assert len(counts) == 4 and all(a == b for a, b in counts), output
    assert output.count("The simulation finished successfully") == 2, output

@pytest.mark.parametrize('wheel_type', ['Wheel0DOF', 'Wheel1DOF_Z'])
@pytest.mark.parametrize('spatial', [False, True])
def test_locked_wheel_spin(tmp_path, wheel_type, spatial):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    extra = ',vertical(wheelC=200000,wheelD=100)' if wheel_type == 'Wheel1DOF_Z' else ''
    (tmp_path/'Locked.mo').write_text(f'''model Locked
  inner parameter Boolean headless=true;
  inner parameter Real linkDiameter=0.02;
  inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false,driveTrainMechanics3D=true);
  Modelica.Mechanics.MultiBody.Joints.Revolute carrier(n={{0,1,0}},useAxisFlange=true);
  Modelica.Mechanics.Rotational.Sources.Speed carrierSpeed(exact=true,w_ref=2);
  Modelica.Mechanics.Rotational.Sources.Torque load(tau=5);
  BobLib.Chassis.Suspension.Tires.Wheels.{wheel_type} wheel(
    includeRotorEffects={str(spatial).lower()},exposeWheelBearing=true,
    pWheel(geometry(R0=0.2,rimR0=0.125,rimWidth=0.18,staticAlpha=0,staticGamma=0){extra}));
equation
  connect(world.frame_b,carrier.frame_a);
  connect(carrierSpeed.flange,carrier.axis);
  connect(carrier.frame_b,wheel.chassisFrame);
  connect(load.flange,wheel.hubFlange);
  assert(abs(wheel.wheelRotSpeedSensor.w)<1e-10,"Locked wheel must not spin relative to its carrier");
  assert(abs(wheel.absoluteSpinSpeed-2)<1e-10,"Locked wheel follows carrier angular motion");
end Locked;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("Locked.mo");
getErrorString();
simulate(Locked,stopTime=0.1);
getErrorString();
''')
    r=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,capture_output=True,check=True,timeout=120)
    output=r.stdout+r.stderr
    assert 'Error:' not in output,output
    assert 'The simulation finished successfully' in output,output
