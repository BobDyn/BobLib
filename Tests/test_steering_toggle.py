"""Check the disabled rack under load, including its chassis reaction."""
import shutil
import subprocess
from pathlib import Path

import pytest


def test_locked_rack_position_and_reaction(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    root = Path(__file__).resolve().parents[1]
    fixture = tmp_path / "LockedRack.mo"
    fixture.write_text('''model LockedRack
  inner parameter Boolean headless = true;
  inner Modelica.Mechanics.MultiBody.World world(g=0, enableAnimation=false);
  BobLib.Chassis.Suspension.Components.RackAndPinion rack(
    steerable=false, lockedRackPosition=0.01, linkDiameter=0.02,
    pRack(leftPickup={0,0.3,0}, cFactor=0.1));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force;
  Modelica.Blocks.Sources.Constant load[3](k={0,100,0});
equation
  connect(world.frame_b, rack.mountFrame);
  connect(force.frame_b, rack.leftFrame);
  connect(load.y, force.force);
  assert(abs(rack.rackAxis.s-0.01)<1e-8, "Rack lock position changed under load");
  assert(abs(abs(rack.mountFrame.f[2])-100)<1e-6, "Rack load did not reach chassis");
  assert(abs(rack.mountFrame.t[3])<1e-6, "Internal pinion reaction did not cancel");
end LockedRack;
''')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{fixture}");
simulate(LockedRack, stopTime=0.01);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, check=True, timeout=120)
    assert "Error:" not in result.stdout, result.stdout
    assert "simulation finished successfully" in result.stdout, result.stdout


def test_four_post_accepts_disabled_front_steering(tmp_path):
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica is not installed")
    root = Path(__file__).resolve().parents[1]
    fixture = tmp_path / "FixedSteeringRig.mo"
    fixture.write_text('''model FixedSteeringRig
  extends BobLib.Experiments.Standards.FourPostSim(
    frAxleDW(steerable=false, lockedRackPosition=0.002));
end FixedSteeringRig;
''')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{fixture}");
checkModel(FixedSteeringRig);
getErrorString();
''')
    result = subprocess.run(["omc", str(script)], cwd=tmp_path,
                            capture_output=True, text=True, check=True, timeout=120)
    assert "Error:" not in result.stdout, result.stdout
    assert "Check of FixedSteeringRig completed successfully" in result.stdout, result.stdout
