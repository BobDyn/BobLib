"""Spatial support torque through the VI mount contract."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_transmission_mount_balance(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "MountedGear.mo"
    fixture.write_text('''model MountedGear
  inner Modelica.Mechanics.MultiBody.World world(g=0, driveTrainMechanics3D=true, enableAnimation=false);
  parameter Real ratio=3;
  parameter Real inputSpeed=30;
  parameter Real efficiency=1;
  parameter Real loadTorque=-10;
  BobLib.Transmissions.FixedRatioTransmission dut(pTransmission(gearRatio=ratio, efficiency=efficiency), usingMultiBodyEngine=true, usingMultiBodyDriveline=true, shaftAxis={0,2,0});
  VehicleInterfaces.PowertrainMounts.SingleSystemRigidMount mount(r_ChassisToMount1={0.2,0.3,0.1});
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true, w_ref=inputSpeed);
  Modelica.Mechanics.Rotational.Sources.ConstantTorque load(tau_constant=loadTorque);
equation
  connect(world.frame_b, mount.chassisFrame);
  connect(mount.mount_1, dut.transmissionMount);
  connect(speed.flange, dut.engineFlange.flange);
  connect(load.flange, dut.drivelineFlange.flange);
  assert(abs(der(dut.drivelineFlange.flange.phi)-inputSpeed/ratio)<1e-9, "Speed ratio");
  assert(abs(dut.engineFlange.flange.tau*inputSpeed + dut.drivelineFlange.flange.tau*inputSpeed/ratio*(if loadTorque*inputSpeed/ratio > 0 then efficiency else 1/efficiency))<1e-8,
    "Gear power balance");
  assert(abs(dut.transmissionMount.t[2]+dut.engineFlange.flange.tau+dut.drivelineFlange.flange.tau)<1e-8,
    "Housing must carry the sum of shaft reactions exactly once");
  assert(sum(dut.transmissionMount.f.^2)<1e-12 and abs(dut.transmissionMount.t[1])+abs(dut.transmissionMount.t[3])<1e-9,
    "No spurious force or transverse torque");
  assert(sum((mount.chassisFrame.t-dut.transmissionMount.t).^2)<1e-12,
    "VI rigid mount must transmit housing torque");
end MountedGear;''')
    cases = [(3,30,-10,1), (3,-30,10,1), (3,30,10,1), (3,30,0,1), (1,30,-10,1), (-2,30,-10,1),
             (3,30,-10,.9), (3,-30,10,.9), (3,30,10,.9), (3,-30,-10,.9), (3,0,0,.9)]
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
''' + "".join(f'''simulate(MountedGear, stopTime=0.1, fileNamePrefix="MountedGear_{index}",
 simflags="-override=ratio={ratio},inputSpeed={speed},loadTorque={load},efficiency={efficiency}");
getErrorString();
''' for index, (ratio, speed, load, efficiency) in enumerate(cases)))
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=360, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    assert output.count("The simulation finished successfully") == len(cases), output


@pytest.mark.parametrize("rotor_effects", [False, True])
def test_motor_stator_and_gyroscopic_reactions(tmp_path, rotor_effects):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "MountedMotor.mo"
    fixture.write_text('''model MountedMotor
  extends BobLibTest.ElectricDrives.TestMotor(dut(usingMultiBodySystem=true, includeRotorEffects=ROTOR_EFFECTS));
  inner Modelica.Mechanics.MultiBody.World world(g=0, driveTrainMechanics3D=true, enableAnimation=false);
  Modelica.Mechanics.MultiBody.Joints.Revolute carrier(n={0,0,1}, useAxisFlange=true);
  Modelica.Mechanics.Rotational.Sources.Speed yaw(exact=true, w_ref=2);
equation
  connect(world.frame_b, carrier.frame_a);
  connect(yaw.flange, carrier.axis);
  connect(carrier.frame_b, dut.chassisFrame);
  assert(abs(dut.chassisFrame.t[2]+dut.shaft_b.flange.tau)<1e-7,
    "Stator and constant-speed shaft reaction must balance");
  assert(abs(dut.chassisFrame.t[1]+(if dut.includeRotorEffects then dut.pMotor.rotorJ*200*2 else 0))<1e-7,
    "Rotor gyroscopic reaction must be included exactly once");
  assert(abs(dut.chassisFrame.t[3])<1e-9 and sum(dut.chassisFrame.f.^2)<1e-12,
    "No spurious motor force or yaw torque");
end MountedMotor;'''.replace('ROTOR_EFFECTS', str(rotor_effects).lower()))
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
simulate(MountedMotor, stopTime=0.05, tolerance=1e-8);
getErrorString();
simulate(BobLibTest.ElectricDrives.TestMotor, stopTime=0.05);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    assert output.count("The simulation finished successfully") == 2, output


@pytest.mark.parametrize("rotor_effects", [False, True])
def test_driveline_mount_and_rotor_reactions(tmp_path, rotor_effects):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "MountedDriveline.mo"
    fixture.write_text('''model MountedDriveline
  extends BobLibTest.Drivelines.TestRearFinalDriveDifferential(
    dut(usingMultiBodyTransmission=true, includeRotorEffects=ROTOR_EFFECTS));
  inner Modelica.Mechanics.MultiBody.World world(g=0, driveTrainMechanics3D=true, enableAnimation=false);
  Modelica.Mechanics.MultiBody.Joints.Revolute carrier(n={0,0,1}, useAxisFlange=true);
  Modelica.Mechanics.Rotational.Sources.Speed yaw(exact=true, w_ref=2);
  Real angularMomentum;
equation
  connect(world.frame_b, carrier.frame_a);
  connect(yaw.flange, carrier.axis);
  connect(carrier.frame_b, dut.chassisFrame);
  angularMomentum=if dut.includeRotorEffects then dut.pDriveline.inputInertia*der(dut.finalDrive.flange_b.phi) +
    dut.pDriveline.halfshaft.inertia*(der(dut.leftHalfshaft.flange_a.phi)+der(dut.rightHalfshaft.flange_a.phi)) else 0;
  assert(abs(dut.chassisFrame.t[1]+2*angularMomentum)<1e-7,
    "All three rotor gyroscopic reactions must reach the mount exactly once");
  assert(abs(dut.chassisFrame.t[2]+dut.finalDrive.flange_a.tau+dut.finalDrive.flange_b.tau)<1e-7,
    "Gear support reaction must reach the VI mount");
  assert(sum(dut.chassisFrame.f.^2)<1e-12 and abs(dut.chassisFrame.t[3])<1e-8,
    "No spurious driveline force or yaw torque");
end MountedDriveline;'''.replace('ROTOR_EFFECTS', str(rotor_effects).lower()))
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("{fixture}");
simulate(MountedDriveline, stopTime=0.1, tolerance=1e-8);
getErrorString();
simulate(BobLibTest.Drivelines.TestRearFinalDriveDifferential, stopTime=0.1);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    assert output.count("The simulation finished successfully") == 2, output


def test_motor_carrier_angular_momentum_and_power(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "FreeMotor.mo"
    fixture.write_text('''model FreeMotor
  inner Modelica.Mechanics.MultiBody.World world(g=0, driveTrainMechanics3D=true, enableAnimation=false);
  BobLib.ElectricDrives.Motor dut(usingMultiBodySystem=true, includeRotorEffects=true);
  Modelica.Electrical.Analog.Sources.ConstantVoltage supply(V=500);
  Modelica.Electrical.Analog.Basic.Ground ground;
  Modelica.Blocks.Sources.Constant power(k=1000);
  Modelica.Mechanics.MultiBody.Joints.Revolute carrier(n={0,1,0},
    phi(start=0,fixed=true), w(start=0,fixed=true));
  Modelica.Mechanics.MultiBody.Parts.Body body(m=1, r_CM={0,0,0},
    I_11=1, I_22=2+dut.pMotor.rotorJ, I_33=1, animation=false);
  Real kineticEnergy;
  Real shaftWork(start=0, fixed=true);
initial equation
  dut.spatialRotor.w=0;
equation
  connect(supply.p, dut.pin_p);
  connect(supply.n, dut.pin_n);
  connect(ground.p, supply.n);
  connect(power.y, dut.controlBus.electricMotorControlBus.availablePower);
  connect(world.frame_b, carrier.frame_a);
  connect(carrier.frame_b, body.frame_a);
  connect(carrier.frame_b, dut.chassisFrame);
  kineticEnergy=0.5*(2+dut.pMotor.rotorJ)*carrier.w^2 +
    dut.pMotor.rotorJ*carrier.w*dut.spatialRotor.w + 0.5*dut.pMotor.rotorJ*dut.spatialRotor.w^2;
  assert(abs((2+dut.pMotor.rotorJ)*carrier.w+dut.pMotor.rotorJ*dut.spatialRotor.w)<1e-6,
    "Free motor must conserve total angular momentum");
  der(shaftWork)=dut.P_mech;
  assert(abs(kineticEnergy-shaftWork)<1e-6,
    "Rotor/carrier kinetic-energy change must equal relative mechanical power");
end FreeMotor;''')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
simulate(FreeMotor, stopTime=0.1, tolerance=1e-9);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    assert "The simulation finished successfully" in output, output


def test_vehicle_separate_mount_locations(tmp_path):
    """Exercise the actual assembly with distinct mount locations and losses."""
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    fixture = tmp_path / "SeparateMountVehicle.mo"
    fixture.write_text('''model SeparateMountVehicle
  extends BobLib.Experiments.Standards.VehicleSim(
    headless=true,
    pVehicle(motor(mountOffset={0.2,0.1,0.05}),
      driveline(mountOffset={-0.3,0,0.08}), transmission(efficiency=0.93)));
equation
  assert(sum((motor.shaft_b.bearingFrame.r_0-chassis.chassisFrame.r_0-
    Modelica.Mechanics.MultiBody.Frames.resolve1(chassis.chassisFrame.R,pVehicle.motor.mountOffset)).^2)<1e-12,
    "Motor stator must follow its own mount location");
  assert(sum((driveline.drivelineMount.r_0-chassis.chassisFrame.r_0-
    Modelica.Mechanics.MultiBody.Frames.resolve1(chassis.chassisFrame.R,pVehicle.driveline.mountOffset)).^2)<1e-12,
    "Differential housing must follow its own mount location");
  assert(sum((motor.shaft_b.bearingFrame.r_0-driveline.drivelineMount.r_0).^2)>0.1,
    "Rotational shaft connection must not collapse the separate spatial mounts");
  assert(rearBrakePlacement==BobLib.Brakes.Types.Placement.Inboard and driveline.useRearInboardBrakes,
    "VehicleSim defaults to rear brakes at the differential outputs");
  assert(abs(brakes.wheelHub_3.flange.phi-driveline.leftHalfshaft.flange_a.phi)<1e-9 and
    abs(brakes.wheelHub_4.flange.phi-driveline.rightHalfshaft.flange_a.phi)<1e-9,
    "Rear brake rotors share the differential output angles before shaft compliance");
  assert(sum((brakes.wheelHub_3.bearingFrame.r_0-driveline.drivelineMount.r_0).^2)<1e-12 and
    sum((brakes.wheelHub_4.bearingFrame.r_0-driveline.drivelineMount.r_0).^2)<1e-12,
    "Rear calipers mount to the sprung differential housing");
end SeparateMountVehicle;''')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
setCommandLineOptions("--numProcs=4 --preOptModules-=evaluateParameters --evaluateFinalParameters=false --evaluateProtectedParameters=false");
simulate(SeparateMountVehicle, stopTime=0.02, tolerance=1e-6);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=600, check=True)
    output = result.stdout + result.stderr
    (tmp_path / "omc.log").write_text(output)
    assert "Error:" not in output, output
    assert "The simulation finished successfully" in output, output

    import xml.etree.ElementTree as ET
    variables = {v.get("name"): v for v in ET.parse(tmp_path / "SeparateMountVehicle_init.xml").iter("ScalarVariable")}
    names = ("pVehicle.motor.mountOffset[1]", "pVehicle.driveline.mountOffset[1]", "pVehicle.transmission.efficiency")
    for name in names:
        assert variables[name].get("isValueChangeable") == "true", name
    rerun = subprocess.run([str(tmp_path / "SeparateMountVehicle"),
        "-override=pVehicle.motor.mountOffset[1]=0.25,pVehicle.driveline.mountOffset[1]=-0.4,pVehicle.transmission.efficiency=0.88",
        "-stopTime=0.02"], cwd=tmp_path, capture_output=True, text=True, timeout=60, check=True)
    assert "The simulation finished successfully" in rerun.stdout + rerun.stderr
