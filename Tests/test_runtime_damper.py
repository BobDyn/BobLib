"""Compiled regressions for runtime curves and fixed-capacity calibration."""
import bisect
import csv
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

BOBLIB_SHARD_BY_TEST = True

ROOT = Path(__file__).resolve().parents[1]


def _build(tmp_path, model, source='', variable_filter='.*', load_test_library=True):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica omc is not installed')
    extra = ''
    if source:
        (tmp_path / 'Fixture.mo').write_text(source)
        extra = 'loadFile("Fixture.mo");'
    test_library = (f'loadFile({json.dumps((ROOT / "Tests/BobLibTest/package.mo").as_posix())});'
                    if load_test_library else '')
    script = f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile({json.dumps((ROOT / 'BobLib/package.mo').as_posix())});
{test_library}
{extra}
setCommandLineOptions("--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none");
buildModel({model}, stopTime=1, numberOfIntervals=200, outputFormat="csv", variableFilter={json.dumps(variable_filter)});
getErrorString();
'''
    (tmp_path / 'build.mos').write_text(script)
    built = subprocess.run(['omc', 'build.mos'], cwd=tmp_path, capture_output=True,
                           text=True, timeout=600)
    (tmp_path / 'build.log').write_text(built.stdout + built.stderr)
    executable = tmp_path / (model + ('.exe' if os.name == 'nt' else ''))
    assert built.returncode == 0 and executable.is_file(), built.stdout + built.stderr
    return executable


def _run(executable, result, overrides):
    arguments = [str(executable), '-r=' + str(result)]
    if overrides:
        arguments.append('-override=' + ','.join(f'{name}={value}' for name, value in overrides.items()))
    completed = subprocess.run(arguments,
        cwd=executable.parent, capture_output=True, text=True, timeout=120)
    result.with_suffix('.log').write_text(completed.stdout + completed.stderr)
    assert completed.returncode == 0 and result.is_file(), completed.stdout + completed.stderr
    with result.open(newline='') as stream:
        return [{name: float(value) for name, value in row.items()}
                for row in csv.DictReader(stream)]


def _force(table, velocity):
    i = max(0, min(bisect.bisect_right([r[0] for r in table], velocity) - 1, len(table) - 2))
    (x0, y0), (x1, y1) = table[i:i + 2]
    return y0 + (velocity - x0)*(y1 - y0)/(x1 - x0)


def test_runtime_curves_change_without_recompiling(tmp_path):
    model = 'BobLibTest.Chassis.Suspension.Components.TestTabularDamperRuntime'
    executable = _build(tmp_path, model)
    digest = hashlib.sha256(executable.read_bytes()).hexdigest()
    tables = {
        'front': [[-.25, -300], [-.1, -200], [0, 0], [.1, 100], [.25, 150]],
        'rear': [[-.25, -500], [0, 0], [.25, 750]],
    }
    path = tmp_path / 'damper curves.txt'
    traces = []
    for run, selected in enumerate(('front', 'rear', 'front')):
        if run == 2:
            tables['front'] = [[-.25, -400], [-.1, -250], [-.05, -90], [0, 0],
                               [.05, 70], [.1, 180], [.25, 250]]
        text = ['#1']
        for name, table in tables.items():
            text.append(f'double {name}({len(table)},2)')
            text.extend(f'{v} {f}' for v, f in table)
        path.write_text('\n'.join(text) + '\n')
        rows = _run(executable, tmp_path / f'run{run}.csv', {
            'useRuntimeDamperTables': 'true', 'damperTableFile': str(path),
            'damperTableName': selected,
        })
        table = tables[selected]
        for row in rows:
            velocity, force = row['compressionVelocity'], row['damperForce']
            assert force == pytest.approx(_force(table, velocity), abs=1e-6)
            assert force*velocity >= -1e-7
            assert bool(row['damper.extrapolating']) == (abs(velocity) > .25)
        assert any(abs(row['compressionVelocity']) > .25 for row in rows)
        assert hashlib.sha256(executable.read_bytes()).hexdigest() == digest
        traces.append([row['damperForce'] for row in rows])
    assert traces[0] != traces[1] and traces[0] != traces[2]


def test_active_rows_ignore_padding_and_accept_two_point_curve(tmp_path):
    source = '''model PaddedDamper
  extends BobLibTest.Chassis.Suspension.Components.TestTabularDamperRuntime(
    damper(damperPoints=3, damperTable=[-.25,-250;0,0;.25,250;0,0;0,0]));
  output Real activePoints=damper.damperPoints;
end PaddedDamper;
'''
    executable = _build(tmp_path, 'PaddedDamper', source)
    digest = hashlib.sha256(executable.read_bytes()).hexdigest()
    for run, overrides in enumerate(({}, {
        'damper.damperPoints': 2, 'damper.damperTable[2,1]': .25,
        'damper.damperTable[1,2]': -500, 'damper.damperTable[2,2]': 500,
    })):
        rows = _run(executable, tmp_path / f'padded{run}.csv', {
            'useRuntimeDamperTables': 'false', **overrides,
        })
        for row in rows:
            velocity = row['compressionVelocity']
            assert row['damperForce'] == pytest.approx((1000 if run == 0 else 2000)*velocity, abs=1e-6)
            assert bool(row['damper.extrapolating']) == (abs(velocity) > .25)
        assert rows[0]['activePoints'] == (3 if run == 0 else 2)
        assert hashlib.sha256(executable.read_bytes()).hexdigest() == digest


def test_existing_bobsim_fixture_loads_without_test_library(tmp_path):
    source = (ROOT / 'Tests/RuntimeTabularDamper.mo').read_text()
    executable = _build(tmp_path, 'RuntimeTabularDamper', source, load_test_library=False)
    table_path = tmp_path / 'compatibility.txt'
    table_path.write_text('#1\ndouble front(3,2)\n-.25 -325\n0 0\n.25 325\n')
    rows = _run(executable, tmp_path / 'compatibility.csv', {
        'useRuntimeDamperTables': 'true', 'damperTableFile': table_path,
    })
    for row in rows:
        assert row['damperForce'] == pytest.approx(1300*row['compressionVelocity'], abs=1e-6)


@pytest.mark.parametrize('law', ['tabular', 'replacement', 'disabled'])
def test_shock_diagnostics_support_replacements_and_disabled_damping(tmp_path, law):
    selection = ('redeclare Modelica.Mechanics.Translational.Components.Damper TabularDamper(d=50),'
                 if law == 'replacement' else '')
    source = f'''model MovingShock
  inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
  Modelica.Mechanics.MultiBody.Joints.Prismatic slider(n={{1,0,0}},useAxisFlange=true);
  Modelica.Mechanics.Translational.Sources.Position motion(exact=true);
  Modelica.Blocks.Sources.Sine command(amplitude=.06,f=1,offset=1);
  BobLib.Chassis.Suspension.Components.ShockLinkage shock(
    {selection}useDamping={'false' if law == 'disabled' else 'true'},s_0=1,
    springTable=[-1,-100;0,0;1,100],damperTable=[-1,-50;0,0;1,50]);
  output Real velocity=shock.damperCompressionVelocity;
  output Real force=shock.damperForce;
equation
  connect(world.frame_b,slider.frame_a);
  connect(world.frame_b,shock.frame_a);
  connect(slider.frame_b,shock.frame_b);
  connect(command.y,motion.s_ref);
  connect(motion.flange,slider.axis);
end MovingShock;
'''
    executable = _build(tmp_path, 'MovingShock', source)
    rows = _run(executable, tmp_path / 'motion.csv', {})
    for row in rows:
        assert row['force'] == pytest.approx(50*row['velocity'], abs=1e-7)
        if law == 'disabled':
            assert row['force'] == 0 and row['velocity'] == 0
    if law != 'disabled':
        assert any(abs(row['velocity']) > .25 for row in rows)


def test_vehicle_routes_runtime_curves_and_diagnostics_to_all_corners(tmp_path):
    # A vertical pulse exercises every damper independently of maneuver settings.
    source = '''model RuntimeVehicle
  extends BobLib.Experiments.Standards.VehicleSim(headless=true);
  Modelica.Mechanics.MultiBody.Forces.WorldForce pulse(
    force={0,0,if time>.05 and time<.15 then 100 else 0});
equation
  connect(pulse.frame_b,chassis.chassisFrame);
end RuntimeVehicle;
'''
    executable = _build(tmp_path, 'RuntimeVehicle', source,
                        variable_filter='time|chassis.damper.*')
    digest = hashlib.sha256(executable.read_bytes()).hexdigest()
    table_path = tmp_path / 'vehicle-dampers.txt'
    for run, rates in enumerate(((1000, 3000), (2000, 6000))):
        table_path.write_text('#1\n' + ''.join(
            f'double {name}(3,2)\n-.25 {-rate*.25}\n0 0\n.25 {rate*.25}\n'
            for name, rate in zip(('front', 'rear'), rates, strict=True)))
        rows = _run(executable, tmp_path / f'vehicle{run}.csv', {
            'useRuntimeDamperTables': 'true', 'damperTableFile': table_path,
        })
        for corner, rate in enumerate((rates[0], rates[0], rates[1], rates[1]), 1):
            velocity = f'chassis.damperCompressionVelocity[{corner}]'
            force = f'chassis.damperForce[{corner}]'
            assert any(abs(row[velocity]) > 1e-6 for row in rows)
            for row in rows:
                assert row[force] == pytest.approx(rate*row[velocity], abs=1e-6)
        assert hashlib.sha256(executable.read_bytes()).hexdigest() == digest
