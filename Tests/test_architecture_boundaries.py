from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


def test_shared_assemblies_accept_an_axle_without_wishbone_members(tmp_path: Path) -> None:
    """Flatten the contract with a deliberately minimal axle, not a dynamics fixture."""
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")
    root = Path(__file__).resolve().parents[1]
    fixture = tmp_path / "NeutralAxleContract.mo"
    fixture.write_text('''package NeutralAxleContract
  model Axle
    extends BobLib.Chassis.Suspension.Interfaces.AxleBase;
    Modelica.Blocks.Sources.Constant load(k=100);
    Modelica.Blocks.Sources.Constant springLength(k=0.3);
  equation
    connect(load.y, axleBus.leftNormalLoad);
    connect(load.y, axleBus.rightNormalLoad);
    connect(springLength.y, axleBus.leftSpringLength);
    connect(springLength.y, axleBus.rightSpringLength);
  end Axle;
  model Chassis
    inner VehicleInterfaces.Roads.FlatRoad road;
    extends BobLib.Chassis.BaseClasses.PartialChassis(
      redeclare Axle frAxleDW,
      redeclare Axle rrAxleDW,
      redeclare BobLib.Chassis.Body.FrameRigid spaceFrame(
        pFrame(frontReference={1,0,0}, rearReference={-1,0,0},
        baseMass(m=1000, rCM={0,0,0}, inertia=identity(3)),
        driverMass(m=0, rCM={0,0,0}, inertia=zeros(3,3)))));
    inner Modelica.Mechanics.MultiBody.World world;
  end Chassis;
  model Rig
    extends BobLib.Experiments.Standards.BaseClasses.PartialFourPostRig(
      redeclare Axle frAxleDW,
      redeclare Axle rrAxleDW,
      frontAxleReference={1,0,0}, rearAxleReference={-1,0,0},
      cpInitFL={1,1,0}, cpInitFR={1,-1,0},
      cpInitRL={-1,1,0}, cpInitRR={-1,-1,0});
  end Rig;
end NeutralAxleContract;
''')
    script = tmp_path / "check.mos"
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("{fixture}");
instantiateModel(NeutralAxleContract.Chassis);
instantiateModel(NeutralAxleContract.Rig);
getErrorString();
''')
    result = subprocess.run([omc, str(script)], text=True, capture_output=True, check=True)
    assert "Error:" not in result.stdout, result.stdout
    assert "class NeutralAxleContract.Chassis" in result.stdout
    assert "class NeutralAxleContract.Rig" in result.stdout
    assert "Wishbone" not in result.stdout
