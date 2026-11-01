"""Differential torque limits, static locking, power, and public integration."""
import pytest
from test_brake_placement import run_model


@pytest.mark.parametrize('kind', ['Open', 'ClutchLSD', 'TorqueBiasing'])
@pytest.mark.parametrize('torque', [200, -200, 0])
def test_sliding_torque_and_power(tmp_path, kind, torque):
    fraction = .35 if torque >= 0 else .15
    ratio = 3 if torque >= 0 else 2
    expected = {'Open': 0, 'ClutchLSD': .85*(20 + fraction*abs(torque)/2),
                'TorqueBiasing': (ratio-1)/(ratio+1)*abs(torque)/2}[kind]
    fixture = tmp_path / 'Sliding.mo'
    fixture.write_text(f'''model Sliding
  BobLib.Drivelines.Internal.Differential1D diff(pDifferential(coastTorqueBiasRatio=2), differentialArchitecture=BobLib.Drivelines.Types.DifferentialArchitecture.{kind});
  Modelica.Mechanics.Rotational.Sources.ConstantTorque inputTorque(tau_constant={torque});
  Modelica.Mechanics.Rotational.Sources.Speed left(exact=true), right(exact=true);
equation
  connect(inputTorque.flange,diff.shaft_in);
  connect(left.flange,diff.shaft_left);
  connect(right.flange,diff.shaft_right);
  left.w_ref=10;
  right.w_ref=20;
  assert(abs(diff.T_lock - {expected}) < 1e-6, "Wrong sliding torque capacity");
  assert(abs(diff.T_left_drive + diff.T_right_drive - ({torque})) < 1e-6, "Torque imbalance");
  assert(diff.lossPower >= -1e-6, "Differential generates energy");
  assert(abs(diff.T_in*diff.w_in - diff.T_left_drive*diff.w_l - diff.T_right_drive*diff.w_r - diff.lossPower) < 1e-6, "Power imbalance");
end Sliding;
''')
    run_model(tmp_path, f'loadFile("{fixture}"); simulate(Sliding, stopTime=0.05);')


@pytest.mark.parametrize('kind', ['ClutchLSD', 'TorqueBiasing', 'Locked'])
def test_static_locking(tmp_path, kind):
    model = 'LockedDifferential1D' if kind == 'Locked' else 'Differential1D'
    modifier = '' if kind == 'Locked' else f'(differentialArchitecture=BobLib.Drivelines.Types.DifferentialArchitecture.{kind})'
    fixture = tmp_path / 'Sticking.mo'
    fixture.write_text(f'''model Sticking
  BobLib.Drivelines.Internal.{model} diff{modifier};
  Modelica.Mechanics.Rotational.Sources.ConstantTorque inputTorque(tau_constant=200);
  Modelica.Mechanics.Rotational.Sources.ConstantTorque leftLoad(tau_constant=-110), rightLoad(tau_constant=-90);
  Modelica.Mechanics.Rotational.Components.Inertia left(J=1, w(start=10,fixed=true), phi(start=0,fixed=true));
  Modelica.Mechanics.Rotational.Components.Inertia right(J=1, w(start=10,fixed=false));
equation
  connect(inputTorque.flange,diff.shaft_in);
  connect(left.flange_a,diff.shaft_left);
  connect(right.flange_a,diff.shaft_right);
  connect(leftLoad.flange,left.flange_b);
  connect(rightLoad.flange,right.flange_b);
  assert(abs(diff.dw)<1e-6, "Static coupling must hold unequal loads without creep");
  assert(abs(diff.T_lock-10)<1e-6, "Static transfer torque incorrect");
end Sticking;
''')
    run_model(tmp_path, f'loadFile("{fixture}"); simulate(Sticking, stopTime=0.1);')


def test_clutch_breakaway(tmp_path):
    fixture = tmp_path / 'Breakaway.mo'
    fixture.write_text('''model Breakaway
  BobLib.Drivelines.Internal.Differential1D diff;
  Modelica.Mechanics.Rotational.Sources.ConstantTorque inputTorque(tau_constant=200);
  Modelica.Mechanics.Rotational.Sources.Torque leftLoad, rightLoad;
  Modelica.Mechanics.Rotational.Components.Inertia left(J=1, w(start=10,fixed=true), phi(start=0,fixed=true));
  Modelica.Mechanics.Rotational.Components.Inertia right(J=1, w(start=10,fixed=false));
equation
  connect(inputTorque.flange,diff.shaft_in);
  connect(left.flange_a,diff.shaft_left);
  connect(right.flange_a,diff.shaft_right);
  connect(leftLoad.flange,left.flange_b);
  connect(rightLoad.flange,right.flange_b);
  leftLoad.tau=if time<0.1 then -110 else -180;
  rightLoad.tau=if time<0.1 then -90 else -20;
  assert(time>=0.09 or abs(diff.dw)<1e-6, "Clutch creeps below breakaway");
  assert(time<=0.15 or (diff.dw < -1 and not diff.locked), "Clutch did not release above capacity");
  assert(diff.lossPower >= -1e-6, "Clutch generates energy");
end Breakaway;
''')
    run_model(tmp_path, f'loadFile("{fixture}"); simulate(Breakaway, stopTime=0.2);')


def test_vehicle_architecture_choices(tmp_path):
    import shutil
    import subprocess

    from test_brake_placement import LOAD
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is not installed')
    script = LOAD
    for kind in ('Open', 'ClutchLSD', 'TorqueBiasing', 'Locked'):
        script += f'''loadString("model Vehicle{kind} extends BobLib.Experiments.Standards.VehicleFMI(driveline(differentialArchitecture=BobLib.Drivelines.Types.DifferentialArchitecture.{kind})); end Vehicle{kind};");
checkModel(Vehicle{kind});
getErrorString();
'''
    mos = tmp_path / 'vehicles.mos'
    mos.write_text(script)
    result = subprocess.run(['omc', str(mos)], cwd=tmp_path, capture_output=True,
                            text=True, check=True, timeout=600)
    assert 'Error:' not in result.stdout, result.stdout
    assert result.stdout.count('completed successfully') == 4, result.stdout
    import re
    counts = re.findall(r'has (\d+) equation\(s\) and (\d+) variable\(s\)', result.stdout)
    assert len(counts) == 4 and all(a == b for a, b in counts), result.stdout


def test_moving_driveline_initialization(tmp_path):
    fixture = tmp_path / 'MovingDriveline.mo'
    fixture.write_text('''model MovingDriveline
  BobLib.Drivelines.RearFinalDriveDifferential driveline(initialOutputAngularVelocity=70);
  Modelica.Mechanics.Rotational.Sources.ConstantTorque inputTorque(tau_constant=0);
  Modelica.Mechanics.Rotational.Components.Inertia left(J=1, w(start=70,fixed=true)), right(J=1, w(start=70,fixed=true));
  Modelica.Mechanics.Rotational.Sources.Torque load;
equation
  connect(inputTorque.flange,driveline.transmissionFlange.flange);
  connect(left.flange_a,driveline.wheelHub_3.flange);
  connect(right.flange_a,driveline.wheelHub_4.flange);
  connect(load.flange,left.flange_b);
  load.tau=if time<0.05 then 0 else -100;
  assert(time>=0.05 or (abs(left.w-70)<1e-6 and abs(right.w-70)<1e-6), "Moving driveline initialized with spurious wheel-speed split");
  assert(time<=0.15 or left.w<right.w, "Driveline failed to release under asymmetric loading");
end MovingDriveline;
''')
    run_model(tmp_path, f'loadFile("{fixture}"); simulate(MovingDriveline, stopTime=0.2);')
