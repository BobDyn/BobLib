"""Driver intent uses one VCU limit/blending path in native and FMI vehicles."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT=Path(__file__).resolve().parents[1]


def test_inverter_disable_restores_friction_braking(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path/'BrakeAvailability.mo').write_text('''model BrakeAvailability
  BobLib.Controllers.VCU dut(enableRegenerativeBraking=true,
    pVCU(finalDriveRatio=3,mechanicalBrakeTorqueLimit=1000,
      regenTorqueLimit=100,regenBrakeBlend=0.5));
  Boolean enabled=time<0.03 or time>=0.07;
equation
  dut.controlBus.driverBus.acceleratorPedalPosition=0;
  dut.controlBus.driverBus.brakePedalPosition=1;
  dut.controlBus.driverBus.inverterEnable=enabled;
  dut.controlBus.electricMotorBus.speed=100;
  assert(abs(dut.mechanicalBrakeTorqueCmd-(if enabled then 700 else 1000))<1e-9,
    "Unavailable regeneration must transfer to friction braking");
  assert(abs(dut.regenBrakeTorqueCmd-(if enabled then 300 else 0))<1e-9,
    "Regeneration must follow inverter availability");
  assert(abs(dut.mechanicalBrakeTorqueCmd-3*dut.tau_cmd_limited-1000)<1e-9,
    "Friction plus enabled motor braking must preserve total demand");
  assert(enabled or abs(dut.P_req)+abs(dut.regenLimitCmd)<1e-9,
    "Disabled inverter must have no regenerative power or torque limit");
end BrakeAvailability;
''')
    (tmp_path/'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("BrakeAvailability.mo");
simulate(BrakeAvailability,stopTime=0.1,numberOfIntervals=100);
getErrorString();
''')
    result=subprocess.run(['omc','run.mos'],cwd=tmp_path,text=True,
                          capture_output=True,timeout=90,check=True)
    output=result.stdout+result.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert 'LOG_ASSERT' not in output,output
    assert 'The simulation finished successfully' in output,output


def test_pedal_limits_and_regenerative_blend(tmp_path):
    if not shutil.which('omc'):
        pytest.skip('OpenModelica is required')
    (tmp_path/'Blend.mo').write_text('''model Blend
  parameter Real accelerator=0;
  parameter Real brake=0.6;
  extends BobLibTest.Controllers.TestVCU(
    vcu(pVCU(finalDriveRatio=3,mechanicalBrakeTorqueLimit=1000,regenTorqueLimit=100,
      regenBrakeBlend=0.5),enableRegenerativeBraking=true),
    acceleratorPedal(k=accelerator),brakePedal(k=brake));
  Real demand=min(1,max(0,brake))*1000;
equation
  assert(abs(vcu.regenBrakeTorqueCmd-min(300,0.5*demand))<1e-10,"Regeneration capacity and demand");
  assert(abs(vcu.mechanicalBrakeTorqueCmd-(demand-vcu.regenBrakeTorqueCmd))<1e-10,"Friction supplies remaining demand");
  assert(abs(vcu.driveTorqueCmd-(min(1,max(0,accelerator))*600-vcu.regenBrakeTorqueCmd))<1e-10,"Driver drive intent and regen share one VCU path");
end Blend;
''')
    script=tmp_path/'run.mos'
    script.write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{ROOT}/BobLib/package.mo");
loadFile("{ROOT}/Tests/BobLibTest/package.mo");
loadFile("Blend.mo");
''' + ''.join(f'''simulate(Blend,stopTime=0.1,simflags="-override=accelerator={a},brake={b}");
getErrorString();
''' for a,b in [(0,.6),(.4,0),(1.2,1.2),(-.2,-.2)]))
    r=subprocess.run(['omc',str(script)],cwd=tmp_path,text=True,capture_output=True,timeout=180,check=True)
    output=r.stdout+r.stderr
    (tmp_path/'omc.log').write_text(output)
    assert 'Error:' not in output,output
    assert output.count('The simulation finished successfully')==4,output
