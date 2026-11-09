"""Boundary safeguards plus optional acceptance of an explicitly supplied SIL FMU."""
import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('sil_export', ROOT / 'Tools/export_fmu.py')
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


def manifest():
    return {direction: {name: dict(specification) for name, specification in
                        exporter.SIL_INTERFACE[direction].items()}
            for direction in ('inputs', 'outputs')}


def test_sil_boundary_rejects_lost_sensors_driver_inputs_and_unit_changes():
    exporter.validate_sil_interface(manifest())
    missing = manifest()
    del missing['outputs']['wheelSpeedRL']
    with pytest.raises(ValueError, match='Missing SIL signal'):
        exporter.validate_sil_interface(missing)
    driver = manifest()
    driver['inputs']['acceleratorPedalCommand'] = {'type': 'Real', 'unit': '1'}
    with pytest.raises(ValueError, match='Unexpected SIL actuator'):
        exporter.validate_sil_interface(driver)
    wrong_unit = manifest()
    wrong_unit['outputs']['motorSpeed']['unit'] = 'rad/s'
    with pytest.raises(ValueError, match='type/unit mismatch'):
        exporter.validate_sil_interface(wrong_unit)


def test_sil_artifact_is_distinct_from_existing_driver_fmu():
    name, source = exporter.vehicle_model('stabar', 'stabar', sil=True)
    original, original_source = exporter.vehicle_model('stabar', 'stabar')
    assert name != original
    assert 'Standards.VehicleSIL(' in source
    assert 'Standards.VehicleFMI(' in original_source
    assert 'initialLongitudinalVelocity=initialVel' in source


@pytest.mark.parametrize('name,value', [('motorTorqueCommand', math.nan),
                                     ('steeringAngleCommand', math.inf),
                                     ('brakeCommand', -.01), ('brakeCommand', 1.01),
                                     ('inverterEnable', 1)])
def test_commands_fail_before_entering_the_solver(name, value):
    pytest.importorskip('fmpy')
    import sil_acceptance
    values = sil_acceptance.commands(0)
    values[name] = value
    with pytest.raises(ValueError):
        sil_acceptance.validate_commands(values)


def test_artifact_hash_mismatch_is_rejected(tmp_path):
    pytest.importorskip('fmpy')
    import sil_acceptance
    archive = tmp_path / 'car.fmu'
    archive.write_bytes(b'corrupted archive')
    (tmp_path / 'manifest.json').write_text(json.dumps({'sha256': 'incorrect'}))
    with pytest.raises(ValueError, match='hash'):
        sil_acceptance.identity(archive)


def test_changed_inputs_include_in_place_scenario_updates():
    pytest.importorskip('fmpy')
    import sil_acceptance

    class Slave:
        def __init__(self):
            self.real = []
            self.boolean = []

        def setReal(self, references, values):
            self.real.append((references, values))

        def setBoolean(self, references, values):
            self.boolean.append((references, values))

    slave = Slave()
    references = dict(zip(sil_acceptance.CONTRACT['inputs'], range(4), strict=True))
    values = sil_acceptance.commands(0)
    previous = sil_acceptance.deliver_inputs(slave, references, values, {})
    previous = sil_acceptance.deliver_inputs(slave, references, values, previous)
    assert len(slave.real) == len(slave.boolean) == 1
    values['brakeCommand'] = .7
    values['inverterEnable'] = False
    previous = sil_acceptance.deliver_inputs(slave, references, values, previous)
    assert slave.real[-1] == ([references['brakeCommand']], [.7])
    assert slave.boolean[-1] == ([references['inverterEnable']], [False])
    values['motorTorqueCommand'] = math.nan
    with pytest.raises(ValueError):
        sil_acceptance.deliver_inputs(slave, references, values, previous)
    assert len(slave.real) == len(slave.boolean) == 2


def test_acceptance_remains_enabled_with_optimized_python():
    pytest.importorskip('fmpy')
    program = '''import sys
sys.path.insert(0, 'Tests')
from sil_acceptance import require
try:
    require(False, 'unsafe result')
except AssertionError:
    sys.exit(0)
sys.exit(1)
'''
    subprocess.run([sys.executable, '-O', '-c', program], cwd=ROOT, check=True, timeout=20)


@pytest.mark.skipif(not os.environ.get('BOBLIB_SIL_FMU'),
                    reason='Set BOBLIB_SIL_FMU to a built car-only FMU for acceptance')
def test_exported_sil_plant(tmp_path):
    pytest.importorskip('fmpy')
    import sil_acceptance
    result = sil_acceptance.acceptance(Path(os.environ['BOBLIB_SIL_FMU']), tmp_path / 'runs')
    assert result['status'] == 'passed'


@pytest.mark.skipif(not os.environ.get('BOBLIB_SIL_FMU'), reason='Set BOBLIB_SIL_FMU to exercise actuator limits')
def test_sil_physical_drive_limits(tmp_path):
    fmpy = pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    import sil_acceptance
    archive = Path(os.environ['BOBLIB_SIL_FMU'])
    variables = {v.name: v for v in fmpy.read_model_description(str(archive)).modelVariables}
    commands = {'motorTorqueCommand': 1000., 'inverterEnable': True,
                'brakeCommand': 0., 'steeringAngleCommand': 0.}
    result = sil_acceptance.run(archive, tmp_path / 'saturated', duration_ms=300,
                                initial_velocity=10., initial_soc=.8, scenario=lambda t: commands)
    assert np.max(result['requestedPower']) > 70000  # No hidden VCU 70 kW cap in the plant.
    assert np.max(result['motorTorqueTarget']) <= float(variables['pVehicle.motor.T_peak'].start) + 1e-5
    assert np.max(result['dcCurrent']) <= float(variables['pVehicle.inverter.I_dc_dis_max'].start) + 1e-5


@pytest.mark.skipif(not os.environ.get('BOBLIB_SIL_FMU'), reason='Set BOBLIB_SIL_FMU to exercise mechanical brakes')
def test_sil_mechanical_brakes_work_with_inverter_disabled(tmp_path):
    pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    import sil_acceptance
    commands = {'motorTorqueCommand': 100., 'inverterEnable': False,
                'brakeCommand': 1., 'steeringAngleCommand': 0.}
    result = sil_acceptance.run(Path(os.environ['BOBLIB_SIL_FMU']), tmp_path / 'disabled_braking',
                                duration_ms=300, initial_velocity=3., scenario=lambda t: commands)
    assert result['velX'][-1] < 2.
    assert np.max(np.abs(result['requestedPower'])) < 1e-8
    assert np.max(np.abs(result['motorTorqueTarget'])) < 1e-8
