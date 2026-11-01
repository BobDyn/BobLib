#!/usr/bin/env python3
"""Validate mirrored coverage and profile fixtures with a locally installed omc.

Example: python3 Tests/modelica_test_library.py --model BobLib.Drivelines --output /tmp/profile
Run without --output to validate coverage only. Output directories must be new.
Rollups sum fixture costs, not exclusive costs within an assembled vehicle.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import re
import shutil
import signal
import subprocess
import tempfile
import time
from pathlib import Path

from modelica_initialization_checks import OMC_COMMAND_LINE_OPTIONS

ROOT = Path(__file__).resolve().parents[1]
PHASES = ('timeFrontend', 'timeBackend', 'timeSimCode', 'timeTemplates',
          'timeCompile', 'timeSimulation', 'timeTotal')


def coverage(root: Path = ROOT) -> list[dict]:
    rows = json.loads((root / 'Tests/model_coverage.json').read_text())['models']
    models = {}
    for path in (root / 'BobLib').rglob('*.mo'):
        if path.name == 'package.mo':
            continue
        match = re.search(r'^\s*((?:(?:encapsulated|partial|expandable|pure|impure|operator|final)\s+)*)(model|block|class|record|function|connector|type|package)\s+(\w+)', path.read_text(), re.M)
        if match and match[2] in ('model', 'block'):
            models['.'.join(path.relative_to(root).with_suffix('').parts)] = 'partial' in match[1].split()
    names = [row['model'] for row in rows]
    if len(names) != len(set(names)) or set(names) != set(models):
        raise ValueError(f'Coverage mismatch: missing={set(models)-set(names)}, stale={set(names)-set(models)}')
    for row in rows:
        parts = row['model'].split('.')[1:]
        expected = '.'.join(['BobLibTest', *parts[:-1], 'Test' + parts[-1]])
        path = root / 'Tests' / (row['test'].replace('.', '/') + '.mo')
        if row['test'] != expected or not path.is_file():
            raise ValueError(f'Invalid mirrored fixture: {row}')
        if row['mode'] not in ('check', 'simulate') or row['scope'] not in ('structural', 'component', 'integration'):
            raise ValueError(f'Invalid test classification: {row}')
        if models[row['model']] and row['mode'] != 'check':
            raise ValueError(f'Partial model cannot be a simulation benchmark: {row}')
        if not math.isfinite(row['stop_time']) or row['stop_time'] <= 0:
            raise ValueError(f'Invalid stop time: {row}')
    return rows


def parse_result(output: str, mode: str, test: str) -> dict:
    if 'Error:' in output:
        return {'status': 'failed'}
    if mode == 'check':
        return {'status': 'passed' if f'Check of {test} completed successfully.' in output else 'failed'}
    result = re.search(r'resultFile\s*=\s*"([^"]+)"', output)
    if not result or 'simulation finished successfully' not in output.lower():
        return {'status': 'failed'}
    timings = {}
    for phase in PHASES:
        match = re.search(r'\b' + phase + r'\s*=\s*([0-9.eE+-]+)', output)
        if not match:
            return {'status': 'failed'}
        try:
            value = float(match[1])
        except ValueError:
            return {'status': 'failed'}
        if not math.isfinite(value) or value < 0:
            return {'status': 'failed'}
        timings[phase] = value
    termination = re.search(r'Simulation call terminate\(\) at time ([0-9.eE+-]+)', output)
    result = {'status': 'passed', 'timings_s': timings}
    if termination:
        result['termination_time'] = float(termination[1])
    return result


def rollups(results: list[dict]) -> dict:
    packages = {}
    for row in results:
        parts = row['model'].split('.')[:-1]
        for depth in range(1, len(parts)+1):
            package = '.'.join(parts[:depth])
            item = packages.setdefault(package, {'passed': 0, 'failed': 0, 'checked': 0,
                'simulated': 0, 'component': 0, 'integration': 0, 'timings_s': {p: 0.0 for p in PHASES}})
            item[row['status']] += 1
            if row['status'] == 'passed':
                if row['mode'] == 'check':
                    item['checked'] += 1
                else:
                    item['simulated'] += 1
                    item[row['scope']] += 1
                    for phase, value in row['timings_s'].items():
                        item['timings_s'][phase] += value
    return packages


def run_case(row: dict, output: Path, timeout: float, root: Path = ROOT) -> dict:
    destination = output / row['test']
    destination.mkdir()
    started = time.monotonic()
    # Native temporary storage avoids Docker bind-mount compilation overhead.
    with tempfile.TemporaryDirectory(prefix='boblib-test-') as temporary:
        build = Path(temporary)
        lines = [f'setCommandLineOptions({json.dumps(OMC_COMMAND_LINE_OPTIONS)});',
                 'setCFlags(getCFlags() + " -Os");', 'loadModel(Modelica, {"4.1.0"});',
                 'loadModel(VehicleInterfaces, {"2.0.2"});',
                 f'loadFile({json.dumps(str(root / "BobLib/package.mo"))});',
                 f'loadFile({json.dumps(str(root / "Tests/BobLibTest/package.mo"))});',
                 f'cd({json.dumps(str(build))});']
        if row['mode'] == 'check':
            lines += [f'checkModel({row["test"]});']
        else:
            lines += [f'(fixtureStart, fixtureStop, fixtureTolerance, fixtureIntervals, fixtureInterval) := getSimulationOptions({row["test"]});']
            lines += [f'simulate({row["test"]}, stopTime={row["stop_time"]}, numberOfIntervals=100, '
                      'tolerance=fixtureTolerance, method="dassl", fileNamePrefix="fixture", simflags="-lv=LOG_STATS");']
        lines += ['getErrorString();']
        script = destination / 'run.mos'
        script.write_text('\n'.join(lines)+'\n')
        with subprocess.Popen(['omc', str(script)], text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, start_new_session=True) as process:
            try:
                log, _ = process.communicate(timeout=timeout)
                result = parse_result(log, row['mode'], row['test'])
                if process.returncode:
                    result = {'status': 'failed', 'reason': f'omc exit {process.returncode}'}
            except subprocess.TimeoutExpired:
                if os.name == 'posix':
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                log, _ = process.communicate()
                result = {'status': 'failed', 'reason': f'timeout after {timeout}s'}
        for makefile in build.glob('*.makefile'):
            shutil.copy2(makefile, destination / makefile.name)
        (destination / 'omc.log').write_text(log)
        result['generated_c_bytes'] = sum(p.stat().st_size for p in build.glob('*.c'))
    return {**row, **result, 'wall_s': time.monotonic()-started}


def write_report(output: Path, results: list[dict], metadata: dict) -> None:
    (output / 'results.json').write_text(json.dumps({**metadata, 'results': results,
        'rollups': rollups(results)}, indent=2)+'\n')
    with (output / 'results.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['model', 'test', 'scope', 'mode', 'stop_time',
            'status', 'wall_s', 'termination_time', 'generated_c_bytes', *PHASES])
        writer.writeheader()
        for row in results:
            writer.writerow({k: v for k, v in {**row, **row.get('timings_s', {})}.items() if k in writer.fieldnames})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='BobLib', help='Production model or package prefix')
    parser.add_argument('--output', type=Path, help='New directory for logs, JSON and CSV')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--timeout', type=float, default=600)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    rows = [r for r in coverage() if r['model'] == args.model or r['model'].startswith(args.model+'.')]
    if not rows:
        parser.error('No models matched')
    print(f'{len(rows)} mirrored fixtures validated', flush=True)
    if args.output is None:
        return 0
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    source_hash = hashlib.sha256()
    for directory in ('BobLib', 'Tests/BobLibTest'):
        for path in sorted((ROOT / directory).rglob('*.mo')):
            source_hash.update(str(path.relative_to(ROOT)).encode()+b'\0'+path.read_bytes())
    metadata = {'omc': subprocess.check_output(['omc', '--version'], text=True).strip(),
                'platform': platform.platform(), 'source_sha256': source_hash.hexdigest(),
                'cflags': '-Os', 'options': OMC_COMMAND_LINE_OPTIONS,
                'note': 'Rollups sum successful fixture costs; integration fixtures repeat shared assemblies.'}
    results = []
    for row in rows:
        if args.check_only:
            row = {**row, 'mode': 'check'}
        result = run_case(row, output, args.timeout)
        results.append(result)
        write_report(output, results, metadata)
        print(f'{result["status"]}: {row["test"]} ({result["wall_s"]:.2f}s)', flush=True)
    return int(any(r['status'] != 'passed' for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
