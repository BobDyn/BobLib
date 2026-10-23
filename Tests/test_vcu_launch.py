"""Low-speed torque/power conversion must agree across the VCU and motor."""
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.parametrize('torque', [20, -20])
def test_rollback_deadband_preserves_requested_torque(tmp_path, torque):
    omc = shutil.which('omc')
    if omc is None:
        pytest.skip('OpenModelica is required')
    root = Path(__file__).resolve().parents[1]
    (tmp_path / 'Rollback.mo').write_text(f'''model Rollback
  BobLib.Controllers.Internal.VCUCore vcu;
  BobLib.ElectricDrives.Internal.PowerLimitedMotor motor(pMotor(w_eps=1,lossTable=[0,0;1000,0],enablePeakTimer=false));
  Modelica.Mechanics.Rotational.Sources.Speed speed(exact=true);
equation
  vcu.cmd_torque_motor={torque};
  vcu.cmd_regen_limit=100;
  vcu.cmd_inverter_enable=true;
  vcu.sens_motor_speed=motor.w;
  speed.w_ref=-2+4*time;
  connect(speed.flange,motor.shaft);
  motor.P_elec=vcu.P_req;
  assert(noEvent(motor.tau_cmd*({torque})>0),
    "Torque changed sign as shaft crossed the low-speed deadband");
end Rollback;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");
loadFile("Rollback.mo");
simulate(Rollback, stopTime=1, numberOfIntervals=1000, tolerance=1e-8);
getErrorString();
''')
    result = subprocess.run([omc, 'run.mos'], cwd=tmp_path, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
    assert result.returncode == 0, result.stdout
    assert 'The simulation finished successfully' in result.stdout, result.stdout
    assert 'LOG_ASSERT' not in result.stdout, result.stdout
