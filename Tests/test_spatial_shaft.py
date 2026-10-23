"""Massless shaft transport conserves spatial moment and physical rotor power."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_spatial_shaft_moment_and_power(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'SpatialShaft.mo'
    fixture.write_text('''model SpatialShaft
  inner Modelica.Mechanics.MultiBody.World world(g=0, enableAnimation=false);
  parameter Real loadTorque=-20;
  parameter Real driveSpeed=30;
  Modelica.Mechanics.MultiBody.Joints.Revolute sprung(n={0,0,1},useAxisFlange=true);
  Modelica.Mechanics.MultiBody.Joints.Revolute upright(n={1,0,0},useAxisFlange=true);
  Modelica.Mechanics.Rotational.Sources.Speed sprungMotion(exact=true,w_ref=1);
  Modelica.Mechanics.Rotational.Sources.Speed uprightMotion(exact=true,w_ref=2);
  Modelica.Mechanics.Rotational.Sources.Speed drive(exact=true,w_ref=driveSpeed);
  Modelica.Mechanics.Rotational.Sources.ConstantTorque load(tau_constant=loadTorque);
  BobLib.Utilities.Mechanics.MultiBody.InboardShaft dut(useSpatialSupports=true);
  Real nIn[3];
  Real nOut[3];
  Real wIn[3];
  Real wOut[3];
  Real tIn[3];
  Real tOut[3];
  Real netMoment[3];
  Real netPower;
equation
  connect(world.frame_b, sprung.frame_a);
  connect(world.frame_b, upright.frame_a);
  connect(sprungMotion.flange, sprung.axis);
  connect(uprightMotion.flange, upright.axis);
  connect(sprung.frame_b, dut.sprungFrame);
  connect(upright.frame_b, dut.uprightFrame);
  connect(drive.flange, dut.inboard);
  connect(load.flange, dut.outboard);
  nIn=Modelica.Mechanics.MultiBody.Frames.resolve1(dut.sprungFrame.R,{0,1,0});
  nOut=Modelica.Mechanics.MultiBody.Frames.resolve1(dut.uprightFrame.R,{0,1,0});
  wIn=Modelica.Mechanics.MultiBody.Frames.angularVelocity1(dut.sprungFrame.R);
  wOut=Modelica.Mechanics.MultiBody.Frames.angularVelocity1(dut.uprightFrame.R);
  tIn=Modelica.Mechanics.MultiBody.Frames.resolve1(dut.sprungFrame.R,dut.sprungFrame.t);
  tOut=Modelica.Mechanics.MultiBody.Frames.resolve1(dut.uprightFrame.R,dut.uprightFrame.t);
  netMoment=dut.inboard.tau*nIn+dut.outboard.tau*nOut+tIn+tOut;
  netPower=dut.inboard.tau*(der(dut.inboard.phi)+nIn*wIn)+
    dut.outboard.tau*(der(dut.outboard.phi)+nOut*wOut)+tIn*wIn+tOut*wOut;
  assert(sum(netMoment.^2)<1e-12,"Massless spatial shaft violates moment balance");
  assert(abs(netPower)<1e-8,"Massless spatial shaft violates physical power balance");
end SpatialShaft;''')
    script = tmp_path / 'check.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
''' + ''.join(f'''simulate(SpatialShaft,stopTime=0.2,tolerance=1e-8,
 simflags="-override=loadTorque={torque},driveSpeed={speed}");
getErrorString();
''' for torque,speed in [(-20,30),(20,-30),(20,30),(0,30)]))
    result = subprocess.run(['omc', str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert output.count('The simulation finished successfully') == 4, output


def test_spatial_caliper_supports(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    fixture = tmp_path / 'SpatialBrakes.mo'
    fixture.write_text('''package SpatialBrakes
  model Wheels
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false,driveTrainMechanics3D=true);
    parameter Boolean differential=false annotation(Evaluate=true);
    BobLib.Brakes.BasicVCUBrakes dut(usingMultiBodyChassis=true,
      useDifferentialInputBrake=differential,pBrakes(differentialBrakeRatio=2));
    Modelica.Blocks.Sources.RealExpression command(y=if time<0.05 then 0 else 1000);
    Modelica.Mechanics.Rotational.Sources.Speed wheels[4](each exact=true,w_ref={10,-10,10,-10});
    Modelica.Mechanics.Rotational.Sources.Speed diffSpeed(exact=true,w_ref=20) if differential;
  equation
    connect(command.y,dut.controlBus.brakesControlBus.mechanicalBrakeTorqueRequest);
''' + ''.join(f'''    connect(wheels[{i}].flange,dut.wheelHub_{i}.flange);
    connect(world.frame_b,dut.wheelHub_{i}.bearingFrame);
    assert(abs(dut.wheelHub_{i}.bearingFrame.t[2]+dut.wheelHub_{i}.flange.tau)<1e-9,
      "Wheel {i} caliper reaction must balance shaft torque");
    assert(dut.wheelHub_{i}.flange.tau*der(dut.wheelHub_{i}.flange.phi)>=-1e-9,
      "Brake must absorb mechanical power in both rotation directions");
''' for i in range(1,5)) + '''    connect(diffSpeed.flange,dut.differentialInput.flange);
    connect(world.frame_b,dut.differentialInput.bearingFrame);
  end Wheels;
  model Diff
    extends Wheels(differential=true);
  equation
    assert(abs(dut.differentialInput.bearingFrame.t[2]+dut.differentialInput.flange.tau)<1e-9,
      "Differential caliper must carry shaft reaction");
    assert(abs(dut.differentialInput.flange.tau-dut.rearCircuitBrakeTorqueRequest/2)<1e-9,
      "Differential brake ratio must apply upstream");
  end Diff;
end SpatialBrakes;
''')
    script = tmp_path / 'check.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{fixture}");
simulate(SpatialBrakes.Wheels,stopTime=0.1);
getErrorString();
simulate(SpatialBrakes.Diff,stopTime=0.1);
getErrorString();
''')
    result = subprocess.run(['omc', str(script)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180, check=True)
    output = result.stdout + result.stderr
    (tmp_path / 'omc.log').write_text(output)
    assert 'Error:' not in output, output
    assert output.count('The simulation finished successfully') == 2, output
