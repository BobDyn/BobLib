#!/usr/bin/env python3
"""Exercise a vehicle FMU without OpenModelica, BobLib sources, or BobSim."""
import argparse
import csv
import json
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from fmpy import read_model_description, simulate_fmu


def native_trajectory(executable: Path, output: Path, values: dict,
                      inputs: np.ndarray, outputs: list[str]) -> tuple[np.ndarray, float]:
    """Run an existing native artifact; no compiler or Modelica sources required."""
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(executable.parent / (executable.stem + '_init.xml'))
    settings = tree.find('DefaultExperiment')
    settings.set('stopTime', str(inputs['time'][-1]))
    settings.set('stepSize', '0.002')
    settings.set('tolerance', '1e-6')
    settings_path = output / 'run_init.xml'
    tree.write(settings_path)
    variables = {v.attrib['name']: v for v in tree.findall('./ModelVariables/ScalarVariable')}
    invalid = [name for name in values if name not in variables or
               variables[name].get('causality') != 'parameter' or
               variables[name].get('isValueChangeable') != 'true']
    if invalid:
        raise ValueError(f'Unknown or non-editable calibration parameters: {invalid}')
    parameters = [n for n in outputs if variables[n].attrib.get('variability') in ('parameter', 'constant')]
    overrides = output / 'overrides.txt'
    overrides.write_text('\n'.join(f'{n}={str(v).lower() if isinstance(v, bool) else v}'
                                    for n, v in values.items())+'\n')
    input_path = output / 'inputs.csv'
    np.savetxt(input_path, np.column_stack([inputs[n] for n in inputs.dtype.names]),
               delimiter=',', header=','.join(inputs.dtype.names), comments='')
    result_path = output / 'trajectory.csv'
    command = [str(executable), f'-f={settings_path}', f'-overrideFile={overrides}',
               f'-csvInput={input_path}', f'-r={result_path}']
    if parameters:
        command.append('-output='+','.join(parameters))
    started = time.perf_counter()
    run = subprocess.run(command, cwd=executable.parent, text=True, capture_output=True, timeout=300)
    elapsed = time.perf_counter()-started
    log = run.stdout+run.stderr
    (output / 'simulation.log').write_text(log)
    if run.returncode or 'simulation finished successfully' not in log:
        raise RuntimeError(f'Native simulation failed; see {output / "simulation.log"}')
    measured = {}
    for name in parameters:
        match = re.search(re.escape(name)+r'\s*=\s*([-+0-9.eE]+)', run.stdout)
        if match is None:
            raise ValueError('Native runtime did not report parameter: ' + name)
        measured[name] = float(match.group(1))
    columns = ['time']+outputs
    with result_path.open() as stream:
        rows = list(csv.DictReader(stream))
    result = np.array([tuple(float(row[n]) if n in row else measured[n] for n in columns)
                       for row in rows], dtype=[(n, np.float64) for n in columns])
    if abs(result['time'][-1]-inputs['time'][-1]) > 1e-10:
        raise ValueError('Native runtime did not honor requested stop time')
    return result, elapsed


def verify(archive: Path, output: Path, native: Path | None = None) -> dict:
    output = output.resolve()
    if native is not None:
        native = native.resolve()
    md = read_model_description(str(archive), validate=True)
    variables = {v.name: v for v in md.modelVariables}
    geometry = 'pVehicle.chassis.frontAxle.geometry.upperFore_i[1]'
    driver_mass = 'pVehicle.chassis.frame.driverMass.m'
    sprung_mass = 'pVehicle.chassis.frame.mass.m'
    total_mass = 'chassis.pTotalMass'
    frame_cg = [f'pVehicle.chassis.frame.mass.rCM[{i}]' for i in range(1, 4)]
    # Mounted-frame aliases may be eliminated; the actual-frame fixture checks their dependency.
    for name in [total_mass, *frame_cg]:
        assert variables[name].causality == 'calculatedParameter', name
    damping = 'pVehicle.driveline.halfshaft.damping'
    stiffness = 'pVehicle.driveline.halfshaft.stiffness'
    inertia = 'pVehicle.driveline.halfshaft.inertia'
    efficiency = 'pVehicle.inverter.eta_mot'
    spring_count = 'pVehicle.chassis.frontAxle.actuation.springPoints'
    aero_count = 'pVehicle.aero.frontPoints'
    architecture_names = ['frontBrakePlacement', 'rearBrakePlacement',
                          'differentialArchitecture']
    for name in (geometry, driver_mass, stiffness, efficiency, spring_count, aero_count):
        if variables[name].causality != 'parameter':
            raise AssertionError(f'{name} is not configurable')
    if variables[damping].causality != 'calculatedParameter':
        raise AssertionError('Halfshaft damping lost its dependency')
    conversions = {'Real': float, 'Integer': int, 'Enumeration': int,
                   'Boolean': lambda x: str(x).lower() == 'true', 'String': str}
    defaults = {v.name: conversions[v.type](v.start) for v in md.modelVariables
                if v.name.startswith('pVehicle.') and v.causality == 'parameter'}
    defaults['initialVel'] = 10.
    base_values = {name: float(variables[name].start)
                   for name in (geometry, driver_mass, stiffness, efficiency)}
    changed = {geometry: base_values[geometry] + 0.001,
               driver_mass: base_values[driver_mass] + 10,
               stiffness: base_values[stiffness] * 1.44,
               efficiency: base_values[efficiency] - 0.03,
               spring_count: 3, aero_count: 4}
    for row, (x, force) in enumerate([(0., 0.), (.1, 4500.), (1., 45000.)], 1):
        changed[f'pVehicle.chassis.frontAxle.actuation.springTable[{row},1]'] = x
        changed[f'pVehicle.chassis.frontAxle.actuation.springTable[{row},2]'] = force
    # Include a scheduled command change at 0.7 s, the previous CVODE failure.
    inputs = np.array([(0., 0., 0., 0.), (.2, 0., 0., 0.),
                       (.7, .05, .1, 0.), (1., .05, .1, 0.)],
                      dtype=[('time', np.float64), ('steeringAngleCommand', np.float64),
                             ('acceleratorPedalCommand', np.float64),
                             ('brakePedalCommand', np.float64)])
    outputs = ['accX', 'accY', 'velX', 'roll', 'Fz_FL', 'Fz_FR', 'Fz_RL', 'Fz_RR',
               geometry, driver_mass, sprung_mass, damping, stiffness, inertia, efficiency, 'inverter.pInverter.eta_mot',
               spring_count, aero_count, total_mass] + frame_cg + architecture_names
    road_friction = 'pVehicle.chassis.frontAxle.tire.force.referenceRoadFriction'
    if road_friction in defaults:
        changed[road_friction] = defaults[road_friction] * 1.1
        outputs.append(road_friction)
    # Corrupt only unused capacity; neither interpolation nor clamping may see it.
    unused = {'pVehicle.chassis.frontAxle.actuation.springTable[64,1]': -999.,
              'pVehicle.chassis.frontAxle.actuation.springTable[64,2]': 999.,
              'pVehicle.aero.frontRideHeightGrid[16]': -999.,
              'pVehicle.aero.dragTable[16,16]': 999.}
    trajectories = {}
    native_checks = []
    output.mkdir(parents=True, exist_ok=True)
    for label, values in [('baseline', {}), ('changed', changed), ('unused_tail', unused)]:
        result = simulate_fmu(str(archive), start_time=0, stop_time=1,
                              output_interval=.002, relative_tolerance=1e-6,
                              start_values=defaults | values, input=inputs, output=outputs,
                              fmi_type='CoSimulation')
        assert result['time'][-1] >= 1
        assert all(np.isfinite(result[name]).all() for name in result.dtype.names)
        for name, value in values.items():
            if name in outputs:
                np.testing.assert_allclose(result[name], value, rtol=0, atol=1e-12)
        np.testing.assert_allclose(result[damping], 2*np.sqrt(result[stiffness]*result[inertia]), rtol=1e-12)
        np.testing.assert_allclose(result['inverter.pInverter.eta_mot'], result[efficiency], rtol=0, atol=1e-12)
        np.save(output / f'{label}.npy', result)
        trajectories[label] = result
        if native is not None:
            native_result, elapsed = native_trajectory(native, output / ('native-'+label),
                                                       defaults | values, inputs, outputs)
            # Native CSV inputs interpolate continuously; CS holds inputs per communication step.
            refined = simulate_fmu(str(archive), start_time=0, stop_time=1,
                                   output_interval=.0001, relative_tolerance=1e-6,
                                   start_values=defaults | values, input=inputs, output=outputs,
                                   fmi_type='CoSimulation')
            comparison = {n: np.interp(result['time'], refined['time'], refined[n])
                          for n in outputs}
            differences = {}
            coarse_differences = {}
            for name in outputs:
                actual = np.interp(result['time'], native_result['time'], native_result[name])
                coarse_differences[name] = float(np.max(np.abs(actual-result[name])))
                differences[name] = float(np.max(np.abs(actual-comparison[name])))
                if variables[name].causality in ('parameter', 'calculatedParameter'):
                    np.testing.assert_allclose(actual, comparison[name], rtol=0, atol=1e-9)
                elif name.startswith('Fz_'):
                    np.testing.assert_allclose(actual, comparison[name], rtol=0, atol=0.5)
                elif name in ('accX', 'accY'):
                    np.testing.assert_allclose(actual, comparison[name], rtol=0, atol=0.01)
                else:
                    np.testing.assert_allclose(actual, comparison[name], rtol=0, atol=1e-4)
            if coarse_differences['velX'] > 1e-4:
                assert differences['velX'] < coarse_differences['velX'] / 4, 'CS input refinement did not converge'
            native_checks.append({'case': label, 'simulation_seconds': elapsed,
                                  'comparison_communication_step': .0001,
                                  'coarse_maximum_difference': coarse_differences,
                                  'maximum_difference': differences})
    baseline, changed_result = trajectories['baseline'], trajectories['changed']
    for name in outputs:
        np.testing.assert_allclose(trajectories['unused_tail'][name], baseline[name],
                                   rtol=1e-10, atol=1e-9)
    np.testing.assert_allclose(changed_result[sprung_mass] - baseline[sprung_mass],
                               10, rtol=0, atol=1e-9)
    np.testing.assert_allclose(changed_result[total_mass] - baseline[total_mass],
                               10, rtol=0, atol=1e-9)
    difference = float(np.max(np.abs(changed_result['Fz_FL'] - baseline['Fz_FL'])))
    assert difference > 1, 'Configuration had no meaningful load effect'
    report = {'fmu': archive.name, 'fmi_version': md.fmiVersion,
              'architecture': {name: next(item.name for item in variables[name].declaredType.items
                                          if int(item.value) == int(baseline[name][0]))
                               for name in architecture_names},
              'duration': 1, 'communication_step': .002,
              'vehicle_parameters_set': len(defaults) - 1,
              'unused_capacity_ignored': True,
              'configuration': changed, 'max_front_left_load_change_N': difference,
              'derived_mass_change_kg': float(changed_result[sprung_mass][0] - baseline[sprung_mass][0]),
              'derived_total_mass_change_kg': float(changed_result[total_mass][0] - baseline[total_mass][0])}
    if native_checks:
        report['native_comparison'] = native_checks
    (output / 'verification.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fmu', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native', type=Path, help='Compare the same compiled architecture using native parameter overrides')
    args = parser.parse_args()
    print(json.dumps(verify(args.fmu, args.output, args.native), indent=2))
