"""VI spatial atmosphere queries follow the aero mount, including rotational velocity."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_spatial_atmosphere_and_offset_velocity(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path/'AtmosphereCase.mo').write_text('''package AtmosphereCase
  model Field
    extends VehicleInterfaces.Atmospheres.ConstantAtmosphere(
      redeclare function density=localDensity,
      redeclare function windVelocity=localWind);
    function localDensity
      extends VehicleInterfaces.Atmospheres.Interfaces.densityBase;
    algorithm
      rho:=1+0.1*r[1];
    end localDensity;
    function localWind
      extends VehicleInterfaces.Atmospheres.Interfaces.windVelocityBase;
    algorithm
      v:={r[1],0,0};
    end localWind;
  end Field;
  model Probe
    extends BobLib.Aero.Interfaces.Base(mountOffset={1,0,0},headless=true);
    output Real position[3]=aeroLoads.frame_b.r_0;
    output Real flowWorld[3]=Modelica.Mechanics.MultiBody.Frames.resolve1(aeroLoads.frame_b.R,relativeAirVelocity);
    output Real rho=airDensity;
  equation
    force=zeros(3);
    torque=zeros(3);
  end Probe;
  model Run
    inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
    inner Field atmosphere;
    Modelica.Mechanics.MultiBody.Parts.Fixed anchor(r={2,0,0});
    Modelica.Mechanics.MultiBody.Joints.Revolute yaw(n={0,0,1},useAxisFlange=true);
    Modelica.Mechanics.Rotational.Sources.Position motion(exact=true,phi_ref=2*time);
    Modelica.Blocks.Sources.Constant heights[4](each k=0.03);
    Probe probe;
  equation
    connect(anchor.frame_b,yaw.frame_a);
    connect(motion.flange,yaw.axis);
    connect(yaw.frame_b,probe.sprungChassisFrame);
    connect(heights[1].y,probe.controlBus.chassisBus.rideHeight_1);
    connect(heights[2].y,probe.controlBus.chassisBus.rideHeight_2);
    connect(heights[3].y,probe.controlBus.chassisBus.rideHeight_3);
    connect(heights[4].y,probe.controlBus.chassisBus.rideHeight_4);
    assert(sum((probe.position-{2+cos(2*time),sin(2*time),0}).^2)<1e-12,"Load-point position");
    assert(abs(probe.rho-(1+0.1*(2+cos(2*time))))<1e-10,"Density queried at load point");
    assert(sum((probe.flowWorld-{-2*sin(2*time)-(2+cos(2*time)),2*cos(2*time),0}).^2)<1e-10,
      "Relative wind includes mount velocity and position-dependent wind");
  end Run;
end AtmosphereCase;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("AtmosphereCase.mo");
getErrorString();
simulate(AtmosphereCase.Run,stopTime=0.5,tolerance=1e-8);
getErrorString();
''')
    run=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,capture_output=True,check=True,timeout=120)
    output=run.stdout+run.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert 'The simulation finished successfully' in output,output
