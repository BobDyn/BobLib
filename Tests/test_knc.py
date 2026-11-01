"""Regression for exact rig motion: linear tables must not supply acceleration."""
import csv
import math
import shutil
import subprocess
from pathlib import Path

import pytest


@pytest.mark.skipif(shutil.which('omc') is None, reason='OpenModelica required')
def test_knc_protocol_derivatives_and_angle_count(tmp_path):
    root = Path(__file__).resolve().parents[1]
    (tmp_path / 'Probe.mo').write_text('''model Probe
  BobLib.Experiments.Standards.BaseClasses.KnCInputs protocol(angleCount=2);
  output Real heave=protocol.heave;
  output Real roll=protocol.roll;
  output Real steer=protocol.steer;
  Modelica.Mechanics.Translational.Sources.Position heaveDrive(exact=true);
  Modelica.Mechanics.Translational.Components.Mass mass(m=1);
  Modelica.Mechanics.Rotational.Sources.Position rollDrive(exact=true);
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=1);
  output Real heaveAcceleration=mass.a;
  output Real rollAcceleration=inertia.a;
equation
  connect(protocol.heave,heaveDrive.s_ref);
  connect(heaveDrive.flange,mass.flange_a);
  connect(protocol.roll,rollDrive.phi_ref);
  connect(rollDrive.flange,inertia.flange_a);
end Probe;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");loadFile("{tmp_path}/Probe.mo");
simulate(Probe,stopTime=237,numberOfIntervals=474,outputFormat="csv",variableFilter="time|heave|roll|steer|heaveAcceleration|rollAcceleration");getErrorString();
''')
    result = subprocess.run(['omc', 'run.mos'], cwd=tmp_path, text=True,
                            capture_output=True, timeout=120)
    assert result.returncode == 0 and 'simulation finished successfully' in result.stdout, result.stdout
    with (tmp_path / 'Probe_res.csv').open() as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    assert all(math.isfinite(v) for row in rows for v in row.values())
    first = next(row for row in rows if abs(row['time'] - 20) < 1e-9)
    second = next(row for row in rows if abs(row['time'] - 139) < 1e-9)
    for row in (first, second):
        assert abs(row['heaveAcceleration']) < 1e-10
        assert abs(row['rollAcceleration']) < 1e-10
    assert first['heave'] == pytest.approx(second['heave'], abs=1e-12)
    assert first['roll'] == pytest.approx(second['roll'], abs=1e-12)
    assert first['steer'] == pytest.approx(-math.pi / 2)
    assert second['steer'] == pytest.approx(-math.pi / 3)


@pytest.mark.skipif(shutil.which('omc') is None, reason='OpenModelica required')
def test_knc_axle_redeclarations(tmp_path):
    root = Path(__file__).resolve().parents[1]
    variants = [('Direct', 'DWDirect', 'DoubleWishboneDirect'),
                ('Bellcrank', 'DWBC', 'DoubleWishboneBellcrank')]
    source = 'package KnCContracts\n'
    for label, tag, axle in variants:
        for mode in ('Steerable', 'Locked'):
            source += f'''model {label}{mode}
  extends BobLib.Experiments.Standards.KnC{mode}(
    redeclare record VehicleRecord=BobLib.Records.VehicleDefn.EVBatInvMotDiff_{tag}_{tag}Record,
    redeclare model AxleModel=BobLib.Chassis.Suspension.{axle});
end {label}{mode};
'''
    for side, rear, mode, axle in [('Fr', False, 'Steerable', 'DoubleWishboneDirect'),
                                    ('Rr', True, 'Locked', 'DoubleWishboneBellcrank')]:
        source += f'''model Mixed{side}
  extends BobLib.Experiments.Standards.KnC{mode}(
    useRearAxle={str(rear).lower()},
    redeclare record VehicleRecord=BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCRecord,
    redeclare model AxleModel=BobLib.Chassis.Suspension.{axle},
    axle(pAxle(actuation=pVehicle.chassis.{'rearAxle' if rear else 'frontAxle'}.actuation)));
end Mixed{side};
'''
    source += 'end KnCContracts;\n'
    (tmp_path / 'KnCContracts.mo').write_text(source)
    names = [f'KnCContracts.{label}{mode}' for label, _, _ in variants
             for mode in ('Steerable', 'Locked')] + ['KnCContracts.MixedFr', 'KnCContracts.MixedRr']
    (tmp_path / 'check.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});loadFile("{root}/BobLib/package.mo");
loadFile("{tmp_path}/KnCContracts.mo");
''' + '\n'.join(f'checkModel({name});getErrorString();' for name in names))
    result = subprocess.run(['omc', 'check.mos'], cwd=tmp_path,
                            capture_output=True, text=True, timeout=180)
    assert result.returncode == 0 and 'Error:' not in result.stdout, result.stdout
    for name in names:
        assert f'Check of {name} completed successfully' in result.stdout


@pytest.mark.skipif(shutil.which('omc') is None, reason='OpenModelica required')
def test_inboard_knc_uses_wheel_boundary(tmp_path):
    root = Path(__file__).resolve().parents[1]
    (tmp_path / 'InboardKnC.mo').write_text('''model InboardKnC
  extends BobLib.Experiments.Standards.KnCLocked(headless=true,
    brakePlacement=BobLib.Brakes.Types.Placement.Inboard);
  equation
    assert(abs(brakes.leftShaft.transmittedTorque-brakes.leftBrake.tau)<1e-7,
      "Rig shaft must transmit the prescribed brake torque");
    when terminal() then
      assert(abs(brakes.leftShaft.transmittedTorque)>10,
        "The longitudinal load must exercise the inboard reaction path");
    end when;
end InboardKnC;
''')
    (tmp_path / 'run.mos').write_text(f'''loadModel(Modelica,{{"4.1.0"}});
loadModel(VehicleInterfaces,{{"2.0.2"}});
loadFile("{root}/BobLib/package.mo");loadFile("{tmp_path}/InboardKnC.mo");
simulate(InboardKnC,stopTime=3,tolerance=1e-6);getErrorString();
''')
    result = subprocess.run(['omc', 'run.mos'], cwd=tmp_path, text=True,
                            capture_output=True, timeout=600, check=True)
    (tmp_path / 'omc.log').write_text(result.stdout+result.stderr)
    assert 'Error:' not in result.stdout, result.stdout
    assert 'simulation finished successfully' in result.stdout, result.stdout
