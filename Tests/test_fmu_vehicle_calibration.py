"""The default vehicle export must keep its calibration editable at runtime."""
import importlib.util
import json
import os
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER_MASS = 'pVehicle.chassis.frame.driverMass.m'
FRAME_MASS = 'pVehicle.chassis.frame.mass.m'
LOADS = [f'Fz_{corner}' for corner in ('FL', 'FR', 'RL', 'RR')]


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to export the vehicle FMU')
def test_default_vehicle_calibration(tmp_path):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')

    spec = importlib.util.spec_from_file_location('vehicle_export', ROOT/'Tools/export_fmu.py')
    exporter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(exporter)
    defaults = exporter.DEFAULTS
    artifact = exporter.build(defaults['front'], defaults['rear'], tmp_path, 4)
    manifest = json.loads((artifact.parent/'manifest.json').read_text())

    assert manifest['source_parameter_inventory']
    assert {DRIVER_MASS, 'pVehicle.chassis.frame.baseMass.m',
            'pVehicle.chassis.frontAxle.actuation.springTable[1,2]',
            'pVehicle.chassis.rearAxle.actuation.springTable[1,2]'} <= manifest['parameters'].keys()
    assert FRAME_MASS in manifest['derived']

    def run(start_values):
        result = fmpy.simulate_fmu(str(artifact), stop_time=.5, output_interval=.25,
                                   start_values=start_values, output=[FRAME_MASS, *LOADS])
        return result[FRAME_MASS][-1], sum(result[name][-1] for name in LOADS)

    added = 50.
    mass, load = run({})
    heavier_mass, heavier_load = run({DRIVER_MASS: float(manifest['parameters'][DRIVER_MASS]['start']) + added})
    assert heavier_mass == pytest.approx(mass + added)
    # Wheel loads have not settled at 0.5 s, so only require most of the added weight.
    assert heavier_load - load > .5 * added * 9.81
