#!/usr/bin/env python3
"""Build the same fixed-architecture, configurable plant as the FMU exporter."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from export_fmu import (
    ARCHITECTURES,
    AXLES,
    DEFAULTS,
    FLAGS,
    ROOT,
    compiler_timings,
    parameter_manifest,
    run,
    source_fingerprint,
    validate_record_inventory,
    vehicle_model,
)


def build(front: str, rear: str, output: Path, *, rotor_effects: bool = False,
          differential: str = DEFAULTS['differential'], front_brakes: str = DEFAULTS['front_brakes'],
          rear_brakes: str = DEFAULTS['rear_brakes']) -> Path:
    name, source = vehicle_model(front, rear, rotor_effects=rotor_effects, differential=differential,
                                 front_brakes=front_brakes, rear_brakes=rear_brakes)
    identity = source_fingerprint(ROOT)
    destination = output / name
    destination.mkdir(parents=True, exist_ok=True)
    flags = FLAGS.replace(' --fmiFlags=s:cvode', '').replace('disableStartCalc', 'disableStartCalc,execstat')
    started = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix='boblib-native-') as temporary:
        work = Path(temporary)
        wrapper = work / f'{name}.mo'
        wrapper.write_text(source)
        script = f'''loadModel(Modelica,{{{json.dumps(ARCHITECTURES["libraries"]["Modelica"])}}});
loadModel(VehicleInterfaces,{{{json.dumps(ARCHITECTURES["libraries"]["VehicleInterfaces"])}}});
loadFile({json.dumps(str(ROOT / 'BobLib/package.mo'))});
loadFile({json.dumps(str(wrapper))});
setCommandLineOptions({json.dumps(flags)});
buildModel({name},stopTime=1,outputFormat="csv",cflags="-Os");
getErrorString();
'''
        (work / 'build.mos').write_text(script)
        compiler_started = time.perf_counter()
        run(['omc', 'build.mos'], work, destination / 'build.log')
        compiler_process_seconds = time.perf_counter()-compiler_started
        log = (destination / 'build.log').read_text()
        executable = work / (name + ('.exe' if platform.system() == 'Windows' else ''))
        if 'Error:' in log or not executable.is_file():
            raise RuntimeError(f'Native build failed; see {destination / "build.log"}')
        manifest = parameter_manifest(work / f'{name}_init.xml')
        for parameter, metadata in manifest['parameters'].items():
            if parameter.startswith('pVehicle.') and metadata.get('isValueChangeable') != 'true':
                raise ValueError('Native calibration is not overrideable: ' + parameter)
        inventory_seconds = validate_record_inventory(name, wrapper, work, manifest, destination)
        if source_fingerprint(ROOT) != identity:
            raise RuntimeError('Sources changed during native build; rebuild from stable sources')
        dependencies = ''
        inspect = ['otool', '-L'] if platform.system() == 'Darwin' else ['ldd']
        if shutil.which(inspect[0]):
            result = subprocess.run(inspect + [str(executable)], text=True, capture_output=True, check=True)
            dependencies = result.stdout
        manifest.update(architecture={'front': front, 'rear': rear, 'differential': differential,
                                      'front_brakes': front_brakes, 'rear_brakes': rear_brakes,
                                      'rotor_effects': rotor_effects,
                                      'fixed_models': ARCHITECTURES['fixed_models']},
                        source=identity, compiler_flags=flags,
                        omc=subprocess.check_output(['omc','--version'],text=True).strip(),
                        platform={'system': platform.system(), 'machine': platform.machine()},
                        runtime_dependencies=dependencies,
                        parameter_lifecycle='Set with -overrideFile before initialization; use a new process per configuration.',
                        total_build_seconds=time.perf_counter()-started)
        artifacts = [executable, wrapper, work / f'{name}_init.xml', work / f'{name}_info.json',
                     *sorted(work.glob(f'{name}*.bin'))]
        manifest['runtime_files'] = [p.name for p in artifacts if p.is_file()]
        manifest['runtime_file_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in artifacts if p.is_file()}
        for artifact in artifacts:
            if artifact.is_file():
                shutil.copy2(artifact, destination / artifact.name)
        settings = ET.parse(destination / f'{name}_init.xml')
        settings.find('DefaultExperiment').set('stopTime', '0')
        settings_path = destination / 'initialization-check_init.xml'
        settings.write(settings_path)
        check_started = time.perf_counter()
        run([str(destination / executable.name), f'-f={settings_path}',
             '-r=initialization-check.csv'], destination, destination / 'initialization-check.log')
        timings = compiler_timings(log, template_phase='Templates')
        timings['calibration_inventory_seconds'] = inventory_seconds
        timings['compiler_process_seconds'] = compiler_process_seconds
        timings['compilation_and_process_overhead_seconds'] = compiler_process_seconds-sum(
            timings[k] for k in ('frontend_seconds','backend_seconds','simcode_seconds','templates_seconds'))
        timings['initialization_check_seconds'] = time.perf_counter()-check_started
        timings['total_build_seconds'] = time.perf_counter()-started
        (destination / 'build-timings.json').write_text(json.dumps(timings, indent=2)+'\n')
        if 'simulation finished successfully' not in (destination / 'initialization-check.log').read_text():
            raise RuntimeError('Relocated native executable failed initialization')
        (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return destination / executable.name


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--all', action='store_true', help='Build all nine default suspension pairs')
    parser.add_argument('--front', choices=AXLES, default=DEFAULTS['front'])
    parser.add_argument('--rear', choices=AXLES, default=DEFAULTS['rear'])
    parser.add_argument('--differential', choices=ARCHITECTURES['differentials'], default=DEFAULTS['differential'])
    parser.add_argument('--front-brakes', choices=ARCHITECTURES['front_brakes'], default=DEFAULTS['front_brakes'])
    parser.add_argument('--rear-brakes', choices=ARCHITECTURES['rear_brakes'], default=DEFAULTS['rear_brakes'])
    parser.add_argument('--rotor-effects', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    pairs = [(front, rear) for front in AXLES for rear in AXLES] if args.all else [(args.front, args.rear)]
    for front, rear in pairs:
        print(build(front, rear, args.output.resolve(), rotor_effects=args.rotor_effects,
                    differential=args.differential, front_brakes=args.front_brakes,
                    rear_brakes=args.rear_brakes), flush=True)


if __name__ == '__main__':
    main()
