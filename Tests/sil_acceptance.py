#!/usr/bin/env python3
"""Prescribed-command acceptance of the car-only FMU; not a firmware harness."""
from __future__ import annotations

import argparse
import contextlib
import csv
import ctypes
import hashlib
import json
import math
import os
import sys
import tempfile
import time
import zipfile
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import get_context
from pathlib import Path

import numpy as np
from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave, defaultCallbacks, fmi2CallbackFunctions, fmi2CallbackLoggerTYPE

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = json.loads((ROOT / 'Tools/sil-interface.json').read_text())
OUTPUTS = list(CONTRACT['outputs'])
REAL_INPUTS = [name for name, spec in CONTRACT['inputs'].items() if spec['type'] == 'Real']


def require(condition, details):
    """Acceptance must remain active even when Python is invoked with -O."""
    if not condition:
        raise AssertionError(details)


@contextlib.contextmanager
def runtime_log(path: Path):
    """Retain native solver stdout/stderr as well as FMI callback messages."""
    sys.stdout.flush()
    sys.stderr.flush()
    libc = ctypes.CDLL(None)
    saved = [os.dup(fd) for fd in (1, 2)]
    try:
        with path.open('wb') as stream:
            for fd in (1, 2):
                os.dup2(stream.fileno(), fd)
            try:
                yield
            finally:
                libc.fflush(None)
    finally:
        for fd, original in zip((1, 2), saved, strict=True):
            os.dup2(original, fd)
            os.close(original)


def commands(milliseconds: int) -> dict:
    """One hold/launch/brake/hold/relaunch/disable/regen sequence, six seconds."""
    t = milliseconds
    torque = 100. if 600 <= t < 1800 or 3600 <= t < 5400 else -30. if t >= 5400 else 0.
    brake = .6 if t < 600 or 1800 <= t < 3600 else 0.
    return {'motorTorqueCommand': torque, 'inverterEnable': not 4800 <= t < 5400,
            'brakeCommand': brake, 'steeringAngleCommand': .05 if t >= 3900 else 0.}


def validate_commands(values: dict) -> None:
    if values.keys() != CONTRACT['inputs'].keys():
        raise ValueError('Supply exactly the four SIL actuator inputs')
    for name, spec in CONTRACT['inputs'].items():
        value = values[name]
        if spec['type'] == 'Boolean':
            if type(value) is not bool:
                raise ValueError(f'{name} must be Boolean')
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        elif value < spec.get('min', -math.inf) or value > spec.get('max', math.inf):
            raise ValueError(f'{name} is outside the actuator range')


def deliver_inputs(slave, references: dict, values: dict, previous: dict) -> dict:
    """Deliver changed commands; unchanged setters invalidate OMC's cached outputs."""
    validate_commands(values)
    changed = [name for name in REAL_INPUTS if name not in previous or values[name] != previous[name]]
    if changed:
        slave.setReal([references[n] for n in changed], [values[n] for n in changed])
    if 'inverterEnable' not in previous or values['inverterEnable'] != previous['inverterEnable']:
        slave.setBoolean([references['inverterEnable']], [values['inverterEnable']])
    return dict(values)


def identity(archive: Path) -> dict:
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    sidecar = archive.parent / 'manifest.json'
    if sidecar.exists() and json.loads(sidecar.read_text()).get('sha256') != digest:
        raise ValueError('FMU hash does not match its build manifest')
    with zipfile.ZipFile(archive) as package:
        metadata = json.loads(package.read('resources/boblib-build.json'))
    if metadata.get('interface') != CONTRACT:
        raise ValueError('FMU does not implement this car-only SIL contract')
    return {'fmu_sha256': digest, 'source_sha256': metadata['source']['sha256'],
            'configuration': {k: metadata[k] for k in
                              ('front', 'rear', 'differential', 'front_brakes', 'rear_brakes')},
            'omc': metadata['omc']}


def run(archive: Path, output: Path, *, step_ms: int = 1, tolerance: float = 1e-6,
        duration_ms: int = 6000, initial_velocity: float = 0., initial_soc: float | None = None,
        scenario=commands) -> np.ndarray:
    """Fresh FMU per run, zero-order-held inputs, integer tick schedule, no retry."""
    if step_ms not in (1, 3) or duration_ms <= 0 or duration_ms % step_ms:
        raise ValueError('Use a 1 or 3 ms exchange and an integral number of steps')
    if not math.isfinite(tolerance) or tolerance <= 0 or not math.isfinite(initial_velocity):
        raise ValueError('Tolerance must be positive and initial velocity finite')
    if initial_soc is not None and (not math.isfinite(initial_soc) or not 0 <= initial_soc <= 1):
        raise ValueError('Initial battery state of charge must be in [0,1]')
    artifact = identity(archive)
    output.mkdir(parents=True, exist_ok=False)
    description = read_model_description(str(archive), validate=True)
    if description.coSimulation is None:
        raise ValueError('FMI Co-Simulation is required')
    variables = {v.name: v for v in description.modelVariables}
    if initial_soc is None:
        initial_soc = float(variables['pVehicle.battery.SOC_start'].start)
    if any(name.startswith('vcu.') for name in variables):
        raise ValueError('A BobLib VCU was compiled into the car-only FMU')
    inputs = {v.name for v in description.modelVariables if v.causality == 'input'}
    if inputs != CONTRACT['inputs'].keys():
        raise ValueError('Unexpected FMU input boundary')
    for direction in ('inputs', 'outputs'):
        for name, spec in CONTRACT[direction].items():
            v = variables[name]
            if v.causality != direction[:-1] or v.type != spec['type'] or v.unit != spec.get('unit'):
                raise ValueError('FMU signal metadata differs: ' + name)
    references = {name: v.valueReference for name, v in variables.items()}
    output_references = [references[name] for name in OUTPUTS]
    columns = ['time', *CONTRACT['inputs'], *OUTPUTS]
    rows = []
    messages = []
    started = time.perf_counter()
    status = {'status': 'failed', **artifact, 'checker_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'step_ms': step_ms, 'tolerance': tolerance,
              'initial_velocity': initial_velocity, 'initial_soc': initial_soc, 'duration_ms': duration_ms}
    initialized = False
    instantiated = False
    slave = None

    def logger(component, instance, level, category, message):
        messages.append(message.decode(errors='replace'))

    with runtime_log(output / 'runtime.log'), tempfile.TemporaryDirectory(prefix='boblib-sil-run-') as directory:
        try:
            slave = FMU2Slave(guid=description.guid, unzipDirectory=extract(str(archive), directory),
                              modelIdentifier=description.coSimulation.modelIdentifier,
                              instanceName='car_only_sil')
            callbacks = fmi2CallbackFunctions()
            callbacks.logger = fmi2CallbackLoggerTYPE(logger)
            callbacks.allocateMemory = defaultCallbacks.allocateMemory
            callbacks.freeMemory = defaultCallbacks.freeMemory
            slave.instantiate(callbacks=callbacks)
            instantiated = True
            slave.setupExperiment(startTime=0., stopTime=duration_ms / 1000, tolerance=tolerance)
            slave.setReal([references['initialVel'], references['pVehicle.battery.SOC_start']],
                          [initial_velocity, initial_soc])
            slave.enterInitializationMode()

            previous_inputs = {}

            def set_inputs(values):
                nonlocal previous_inputs
                previous_inputs = deliver_inputs(slave, references, values, previous_inputs)

            values = scenario(0)
            set_inputs(values)
            slave.exitInitializationMode()
            initialized = True
            for milliseconds in range(0, duration_ms, step_ms):
                values = scenario(milliseconds)
                set_inputs(values)
                measured = slave.getReal(output_references)
                if not all(math.isfinite(value) for value in measured):
                    raise RuntimeError(f'Nonfinite output at {milliseconds} ms')
                rows.append([milliseconds / 1000, *(values[n] for n in CONTRACT['inputs']), *measured])
                # An error/discard/exception ends the run; never reuse its state.
                slave.doStep(currentCommunicationPoint=milliseconds / 1000,
                             communicationStepSize=step_ms / 1000)
            measured = slave.getReal(output_references)
            if not all(math.isfinite(value) for value in measured):
                raise RuntimeError('Nonfinite final output')
            rows.append([duration_ms / 1000, *(values[n] for n in CONTRACT['inputs']), *measured])
            status['status'] = 'passed'
        except Exception as error:
            status['error'] = f'{type(error).__name__}: {error}'
            raise
        finally:
            if slave is not None and instantiated:
                try:
                    try:
                        if initialized and status['status'] == 'passed':
                            slave.terminate()
                    finally:
                        slave.freeInstance()
                except Exception as error:
                    status['status'] = 'failed'
                    status['cleanup_error'] = str(error)
            status['wall_seconds'] = time.perf_counter() - started
            status['completed_seconds'] = rows[-1][0] if rows else 0
            status['wall_per_simulated_second'] = (status['wall_seconds'] / status['completed_seconds']
                                                 if status['completed_seconds'] else None)
            (output / 'run.json').write_text(json.dumps(status, indent=2) + '\n')
            (output / 'fmi.log').write_text('\n'.join(messages) + '\n')
            with (output / 'signals.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(columns)
                writer.writerows(rows)
    if status['status'] != 'passed':
        raise RuntimeError('FMU cleanup failed; see run.json')
    return np.array([tuple(row) for row in rows], dtype=[(name, float) for name in columns])


def grade(result: np.ndarray) -> dict:
    def at(name, t):
        return float(np.interp(t, result['time'], result[name]))

    held = (result['time'] >= 3.3) & (result['time'] < 3.6)
    disabled = (result['time'] >= 4.82) & (result['time'] < 5.4)
    regen = result['time'] >= 5.5
    metrics = {'first_launch_speed': at('velX', 1.8),
               'held_speed_max': float(np.max(np.abs(result['velX'][held]))),
               'relaunch_speed': at('velX', 4.8),
               'regen_speed_drop': at('velX', 5.4) - at('velX', 6.),
               'regen_current_min': float(np.min(result['dcCurrent'][regen])),
               'turn_acceleration_max': float(np.max(result['accY'][result['time'] >= 4.2]))}
    require(metrics['first_launch_speed'] > 1., metrics)
    require(metrics['held_speed_max'] < .05, metrics)
    require(metrics['relaunch_speed'] > 1., metrics)
    require(np.max(np.abs(result['requestedPower'][disabled])) < 1e-8, 'Disabled drive requested power')
    require(np.max(np.abs(result['motorTorqueTarget'][disabled])) < 1e-8, 'Disabled drive requested torque')
    require(metrics['regen_current_min'] < -.1, metrics)
    require(metrics['regen_speed_drop'] > .1, metrics)
    require(metrics['turn_acceleration_max'] > .01, metrics)
    require(np.min(result['dcVoltage']) > 0, 'Nonpositive DC bus voltage')
    return metrics


def acceptance(archive: Path, output: Path, *, quick: bool = False, jobs: int = 1) -> dict:
    """Report engineering smoke limits, repeatability and sensitivity, not certification."""
    if jobs < 1:
        raise ValueError('Acceptance jobs must be positive')
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    report = {'status': 'failed', **identity(archive), 'profile_workers': min(jobs, 3), 'runs': {}}
    try:
        baseline = run(archive, output / 'baseline')
        report['runs']['baseline'] = grade(baseline)
        if not quick:
            def constant(torque, enable):
                return lambda t: {'motorTorqueCommand': torque, 'inverterEnable': enable,
                                  'brakeCommand': 0., 'steeringAngleCommand': 0.}

            disabled = run(archive, output / 'disabled_standstill', duration_ms=300,
                           scenario=constant(100., False))
            require(np.max(np.abs(disabled['velX'])) < .05, 'Disabled standstill moved')
            require(np.max(np.abs(disabled['requestedPower'])) < 1e-8, 'Disabled standstill requested power')
            full_pack = run(archive, output / 'full_charge_regen', duration_ms=300,
                            initial_velocity=3., initial_soc=1., scenario=constant(-30., True))
            variables = {v.name: v for v in read_model_description(str(archive)).modelVariables}
            voltage_limit = float(variables['pVehicle.inverter.V_dc_max'].start)
            require(np.max(full_pack['dcVoltage']) <= voltage_limit + 1e-4, 'Full-pack overvoltage')
            require(np.min(full_pack['dcCurrent']) > -.01, 'Full pack accepted charging current')
            for label, torque in [('reverse_motoring', -30.), ('reverse_regen', 30.)]:
                reverse = run(archive, output / label, duration_ms=300, initial_velocity=-3.,
                              initial_soc=.8, scenario=constant(torque, True))
                if torque < 0:
                    require(reverse['velX'][-1] < -3., 'Reverse motoring did not accelerate')
                    require(np.mean(reverse['dcCurrent'][100:]) > .1, 'Reverse motoring did not discharge')
                else:
                    require(reverse['velX'][-1] > -3., 'Reverse regen did not decelerate')
                    require(np.mean(reverse['dcCurrent'][100:]) < -.1, 'Reverse regen did not charge')
            profiles = [('repeat', 1, 1e-6), ('exchange_3ms', 3, 1e-6), ('tolerance_1e7', 1, 1e-7)]

            def check_profile(label, result):
                if label == 'repeat':
                    for name in OUTPUTS:
                        np.testing.assert_allclose(result[name], baseline[name], rtol=1e-8, atol=1e-8)
                    report['runs'][label] = {'status': 'passed'}
                    return
                metrics = grade(result)
                # Fixed before testing: 0.25 m/s maximum speed difference against
                # the 1 ms / 1e-6 reference, including command transients.
                error = float(np.max(np.abs(result['velX'] - np.interp(
                    result['time'], baseline['time'], baseline['velX']))))
                metrics['max_speed_difference'] = error
                report['runs'][label] = metrics
                require(error < .25, (label, error))
            if jobs == 1:
                for label, step, tolerance in profiles:
                    check_profile(label, run(archive, output / label, step_ms=step, tolerance=tolerance))
            else:
                # Spawn fresh processes: OpenModelica globals and native log
                # redirection must never be shared between concurrent runs.
                with ProcessPoolExecutor(max_workers=min(jobs, 3), mp_context=get_context('spawn')) as pool:
                    futures = {label: pool.submit(run, archive, output / label, step_ms=step,
                                                  tolerance=tolerance)
                               for label, step, tolerance in profiles}
                    for label, future in futures.items():
                        check_profile(label, future.result())
        report['status'] = 'passed'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        report['wall_seconds'] = time.perf_counter() - started
        (output / 'acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fmu', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory for logs and report')
    parser.add_argument('--quick', action='store_true', help='One 6 s run, without repeat/sensitivity runs')
    parser.add_argument('--jobs', type=int, default=1,
                        help='Independent profile processes (at most 3); every run keeps its own FMU instance')
    args = parser.parse_args()
    print(json.dumps(acceptance(args.fmu.resolve(), args.output.resolve(), quick=args.quick,
                                jobs=args.jobs), indent=2))


if __name__ == '__main__':
    main()
