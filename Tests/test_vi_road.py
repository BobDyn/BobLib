"""VI road projection, normal reaction and friction affect the actual tire load path."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]


def test_road_physics(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path/'RoadCases.mo').write_text('''package RoadCases
  model Probe
    parameter BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle
      "Complete calibration supplying the probe's tire-force record";
    extends BobLib.Chassis.Suspension.Tires.BaseTire(
      pTire(force=pVehicle.chassis.frontAxle.tire.force),
      useRoadSurface=true,realExpressionFy(y=1000),realExpressionMz(y=100));
    output Real appliedForceWorld[3]=tireForceWorld;
    output Real appliedTorqueWorld[3]=tireTorqueWorld;
  end Probe;
  model FlatFast
    extends BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadGround(dut(flatRoadFastPath=true));
  end FlatFast;
  model InvalidFlatFast
    inner BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadSurface.CurvedRoad road;
    BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadSurface surface(flatRoadFastPath=true,r={2,3,0});
  end InvalidFlatFast;
  model CurvedContact
    extends BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadGround(
      redeclare BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadSurface.CurvedRoad road);
  end CurvedContact;
  model MovingContact
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
    inner BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadSurface.CurvedRoad road;
    Real coordinate=2+time;
    Real gap=-0.01+0.005*time;
    Real root=sqrt(1+0.01*coordinate^2);
    Real pen=0.5*(sqrt(gap^2+1e-12)-gap);
    Real penRate=0.0025*(gap/sqrt(gap^2+1e-12)-1);
    Real raw=100000*pen+750*penRate;
    Modelica.Mechanics.MultiBody.Joints.Prismatic x(n={1,0,0},useAxisFlange=true);
    Modelica.Mechanics.MultiBody.Joints.Prismatic z(n={0,0,1},useAxisFlange=true);
    Modelica.Mechanics.Translational.Sources.Position px(exact=true,s_ref=coordinate-gap*0.1*coordinate/root);
    Modelica.Mechanics.Translational.Sources.Position pz(exact=true,s_ref=0.05*coordinate^2+gap/root);
    BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround ground(r={2,0,0});
  equation
    connect(world.frame_b,x.frame_a);
    connect(x.frame_b,z.frame_a);
    connect(px.flange,x.axis);
    connect(pz.flange,z.axis);
    connect(z.frame_b,ground.frame_b);
    assert(abs(ground.surface.gap-gap)<1e-8,"Moving curved surface projection");
    assert(abs(ground.f_z-0.5*(sqrt(raw^2+1)+raw))<1e-5,"Normal damping uses the correct curved projection derivative");
  end MovingContact;
  model Friction
    inner parameter Boolean headless=true;
    inner parameter Real linkDiameter=0.02;
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
    inner BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadSurface.CurvedRoad road(mu=0.25);
    Modelica.Mechanics.MultiBody.Parts.Fixed anchor(r={2,0,0.5});
    Probe tire(
      pTire(wheel(geometry(R0=0.3,rimR0=0.2,rimWidth=0.2,staticAlpha=0,staticGamma=0))));
  equation
    connect(anchor.frame_b,tire.chassisFrame);
    assert(abs(sqrt(tire.appliedForceWorld*tire.appliedForceWorld)-500)<1e-8,"Road coefficient halves calibrated shear force");
    assert(abs(sqrt(tire.appliedTorqueWorld*tire.appliedTorqueWorld)-50)<1e-8,"Road coefficient halves aligning moment");
    assert(abs(tire.appliedForceWorld*tire.appliedTorqueWorld)<1e-8,"Shear force tangent and aligning moment normal");
    assert(abs(tire.appliedTorqueWorld[1])>1,"Curved road normal changes moment direction");
  end Friction;
end RoadCases;
''')
    names=['BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadSurface',
           'BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics.TestRoadGround',
           'RoadCases.CurvedContact','RoadCases.Friction','RoadCases.MovingContact','RoadCases.FlatFast']
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("RoadCases.mo");
getErrorString();
'''+''.join(f'simulate({name},stopTime=0.1);\ngetErrorString();\n' for name in names))
    r=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,capture_output=True,check=True,timeout=180)
    output=r.stdout+r.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert output.count('The simulation finished successfully')==len(names),output

    (tmp_path/'reject.mos').write_text((tmp_path/'run.mos').read_text().split('simulate(')[0]+
        'simulate(RoadCases.InvalidFlatFast,stopTime=0.1);\ngetErrorString();\n')
    rejected=subprocess.run(['omc','reject.mos'],cwd=tmp_path,text=True,capture_output=True,check=True,timeout=60)
    assert 'The simulation finished successfully' not in rejected.stdout
    assert 'flatRoadFastPath requires' in rejected.stdout
