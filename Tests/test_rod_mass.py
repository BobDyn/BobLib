"""Rod mass must load both pickups without adding a free articulation."""
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_rod_mass_reactions_and_inertia(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica required')
    (tmp_path/'MassCases.mo').write_text('''package MassCases
  model Static
    inner Modelica.Mechanics.MultiBody.World world(n={0,0,-1},g=10,enableAnimation=false);
    Modelica.Mechanics.MultiBody.Parts.Fixed left;
    Modelica.Mechanics.MultiBody.Joints.Prismatic right(n={1,0,0},s(start=1));
    BobLib.Chassis.Suspension.Components.Rod rod(r_a={0,0,0},r_b={1,0,0},
      n1_a={0,1,0},useMass=true,pRod(m=2,rCM={0.4,0,0},inertia=diagonal({0.1,0.2,0.3})),
      linkDiameter=0.02,jointDiameter=0.03);
  equation
    connect(left.frame_b,rod.frame_a);
    connect(world.frame_b,right.frame_a);
    connect(right.frame_b,rod.frame_b);
    assert(abs(left.frame_b.f[3]+12)<1e-8,"Inboard pickup carries 60 percent of rod weight");
    assert(abs(world.frame_b.f[3]+8)<1e-8,"Outboard pickup carries 40 percent of rod weight");
  end Static;
  model Rotating
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
    Modelica.Mechanics.MultiBody.Joints.Revolute pivot(n={0,1,0},useAxisFlange=true);
    Modelica.Mechanics.Rotational.Sources.Position drive(exact=true,phi_ref=0.5*time*time);
    Modelica.Mechanics.MultiBody.Joints.Prismatic right(n={1,0,0},s(start=1));
    BobLib.Chassis.Suspension.Components.Rod rod(r_a={0,0,0},r_b={1,0,0},
      n1_a={0,1,0},useMass=true,pRod(m=2,rCM={0.4,0,0},inertia=diagonal({0.1,0.2,0.3})),
      linkDiameter=0.02,jointDiameter=0.03);
  equation
    connect(world.frame_b,pivot.frame_a);
    connect(drive.flange,pivot.axis);
    connect(pivot.frame_b,rod.frame_a);
    connect(pivot.frame_b,right.frame_a);
    connect(right.frame_b,rod.frame_b);
    assert(abs(drive.flange.tau+0.52)<1e-8,"Drive sees CG inertia plus parallel-axis contribution");
  end Rotating;
end MassCases;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");loadFile("MassCases.mo");
simulate(MassCases.Static,stopTime=0.1,tolerance=1e-9);getErrorString();
simulate(MassCases.Rotating,stopTime=0.1,tolerance=1e-9);getErrorString();
''')
    result=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,capture_output=True,timeout=120,check=True)
    output=result.stdout+result.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert 'LOG_ASSERT' not in output,output
    assert output.count('The simulation finished successfully')==2,output
    for case in ('Static', 'Rotating'):
        metadata = ET.parse(tmp_path / f'MassCases.{case}_init.xml').getroot()
        assert int(metadata.attrib['numberOfContinuousStates']) == 0
