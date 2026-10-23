#!/usr/bin/env python3
"""Build an FMI 2 vehicle with pre-initialization calibration (ME and CS)."""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import platform
import re
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ARCHITECTURES = json.loads((ROOT / 'Tools/architectures.json').read_text())
DEFAULTS = ARCHITECTURES['defaults']
AXLES = {key: (value['tag'], value['model']) for key, value in ARCHITECTURES['axles'].items()}
CURVE_CAPACITY = ARCHITECTURES['table_capacities']['curve_points']
AERO_CAPACITY = ARCHITECTURES['table_capacities']['aero_axis_points']
# Preserve numeric calibration through dependent MSL parameters. Structural
# conditional components and array dimensions still specialize at translation.
FLAGS = ('--ignoreCommandLineOptionsAnnotation=true '
         '--ignoreSimulationFlagsAnnotation=true '
         '--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection '
         '-d=NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 '
         '--generateDynamicJacobian=none --homotopyApproach=equidistantLocal '
         '--preOptModules-=evaluateParameters --evaluateFinalParameters=false '
         '--evaluateProtectedParameters=false --fmiFlags=s:cvode '
         '--tearingStrictness=veryStrict --dynamicTearing=linear '
         '--dynamicTearingForInitialization=true')
DERIVED = ('pVehicle.chassis.frame.mass.',) + tuple(
    f'pVehicle.chassis.{axle}.tire.wheel.compliance.{kind}Damping['
    for axle in ('frontAxle', 'rearAxle') for kind in ('translation', 'rotation'))
DERIVED_NAMES = {
    f'pVehicle.chassis.{axle}.tire.force.{section}.UNLOADED_RADIUS'
    for axle in ('frontAxle', 'rearAxle') for section in ('setup', 'relaxation')
} | {
    'pVehicle.battery.Q_cell',
    'pVehicle.brakes.differentialBrakeRatio', 'pVehicle.brakes.maxTorque',
    'pVehicle.controller.finalDriveRatio',
    'pVehicle.chassis.frame.torsionalDamping',
    'pVehicle.driveline.halfshaft.damping',
    'pVehicle.driveline.frontBrakeHalfshaft.damping',
} | {
    f'pVehicle.chassis.frame.{axle}Reference[{axis}]'
    for axle in ('front', 'rear') for axis in (1, 3)
} | {
    f'pVehicle.battery.{field}[{index}]'
    for field in ('R0_chg_cell_table', 'R0_dis_cell_table',
                  'R1_chg_cell_table', 'R1_dis_cell_table')
    for index in range(1, 8)
} | {
    f'pVehicle.inverter.eta_{mode}_table[{index}]'
    for mode in ('mot', 'reg') for index in range(1, 7)
}


def source_fingerprint(root: Path) -> dict:
    """Identify dirty working-tree sources as well as committed revisions."""
    paths = sorted(p for p in (root / 'BobLib').rglob('*')
                   if p.is_file() and (p.suffix == '.mo' or p.name == 'package.order'))
    paths += sorted((root / 'Tools').glob('*.py'))
    paths += sorted((root / 'Tools').glob('*.json'))
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in paths}
    return {'sha256': hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
            'files': hashes}


def vehicle_model(front: str, rear: str, *, rotor_effects: bool = False,
                  differential: str = DEFAULTS['differential'], front_brakes: str = DEFAULTS['front_brakes'],
                  rear_brakes: str = DEFAULTS['rear_brakes']) -> tuple[str, str]:
    for key, value in [('differentials', differential), ('front_brakes', front_brakes), ('rear_brakes', rear_brakes)]:
        if value not in ARCHITECTURES[key]:
            raise ValueError(f'Unsupported {key}: {value}')
    fr, fr_model = AXLES[front]
    rr, rr_model = AXLES[rear]
    name = f'BobLibVehicle_{fr}_{rr}'
    if rotor_effects:
        name += '_RotorEffects'
    if differential != 'ClutchLSD':
        name += '_' + differential
    if front_brakes != 'Outboard':
        name += '_Front' + front_brakes
    if rear_brakes != 'Outboard':
        name += '_Rear' + rear_brakes
    record = f'BobLib.Records.VehicleDefn.EVBatInvMotDiff_{fr}_{rr}Record'
    calibration = []
    axles = []
    for axle in ('frontAxle', 'rearAxle'):
        base = f'defaultVehicle.chassis.{axle}.actuation'
        curves = []
        for kind in ('spring', 'damper'):
            table = f'{base}.{kind}Table'
            curves += [f'{kind}Points=size({table}, 1)',
                       f'{kind}Table=cat(1, {table}, zeros({CURVE_CAPACITY}-size({table}, 1), 2))']
        axles.append(f'{axle}(actuation(' + ', '.join(curves) + '))')
    calibration.append('chassis(' + ', '.join(axles) + ')')
    aero = []
    for axis in ('front', 'rear'):
        grid = f'defaultVehicle.aero.{axis}RideHeightGrid'
        aero += [f'{axis}Points=size({grid}, 1)',
                 f'{axis}RideHeightGrid=cat(1, {grid}, zeros({AERO_CAPACITY}-size({grid}, 1)))']
    for field in ('dragTable', 'downforceTable', 'mxTable', 'myTable', 'mzTable'):
        table = f'defaultVehicle.aero.{field}'
        aero.append(f'{field}=cat(1, cat(2, {table}, zeros(size({table}, 1), '
                    f'{AERO_CAPACITY}-size({table}, 2))), '
                    f'zeros({AERO_CAPACITY}-size({table}, 1), {AERO_CAPACITY}))')
    calibration.append('aero(' + ', '.join(aero) + ')')
    calibration = ',\n      '.join(calibration)
    return name, f'''model {name}
  constant {record} defaultVehicle;
  extends BobLib.Experiments.Standards.VehicleFMI(
    final headless=true,
    final differentialArchitecture=BobLib.Drivelines.Types.DifferentialArchitecture.{differential},
    final includeRotorEffects={str(rotor_effects).lower()},
    final frontBrakePlacement=BobLib.Brakes.Types.Placement.{front_brakes},
    final rearBrakePlacement=BobLib.Brakes.Types.Placement.{rear_brakes},
    pVehicle({calibration}),
    redeclare record VehicleRecord = {record},
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      final flatRoadFastPath=true,
      redeclare record ChassisRecord = {record}.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.{fr_model},
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.{rr_model},
      headless=headless,
      initialLongitudinalVelocity=initialVel,
      pChassis = pVehicle.chassis));
end {name};
'''


def split_initializer(source: str, name: str) -> str:
    """Bound Clang's work per function without altering metadata assignments."""
    signature = f'void {name}_read_input_fmu(MODEL_DATA* modelData)'
    head, tail = source.split(signature, 1)
    if tail.endswith('}'):
        tail += '\n'
    if not tail.startswith('\n{\n') or not tail.endswith('}\n'):
        raise ValueError('Unsupported OMC FMI initializer layout')
    body = tail[3:-2].splitlines(keepends=True)
    if not all(line.rstrip().endswith(';') and line.lstrip().startswith(
            ('modelData->', 'put_real_element(')) for line in body):
        raise ValueError('Initializer contains more than metadata assignments')
    chunks = [body[i:i+2000] for i in range(0, len(body), 2000)]
    result = head
    for i, chunk in enumerate(chunks):
        result += (f'static void metadata_chunk_{i}(MODEL_DATA* modelData) {{\n'
                   + ''.join(chunk) + '}\n')
    result += signature + ' {\n' + ''.join(
        f'  metadata_chunk_{i}(modelData);\n' for i in range(len(chunks))) + '}\n'
    restored = [line for line in result.splitlines(keepends=True)
                if line.lstrip().startswith(('modelData->', 'put_real_element('))]
    if restored != body:
        raise ValueError('Initializer rewrite changed assignment order')
    return result


def guard_cvode_events(source: str) -> str:
    """Work around OMC 1.27.1 passing roundoff-sized intervals to CVODE."""
    needle = ('  solverInfo->currentTime = data->localData[0]->timeValue;\n\n'
              '  N_VSetArrayPointer(states, cvodeData->y);')
    if source.count(needle) != 1:
        raise ValueError('Unsupported OMC CVODE source; review the event workaround')
    replacement = ('  solverInfo->currentTime = data->localData[0]->timeValue;\n\n'
                   '  /* Do not integrate a roundoff-sized FMI event interval. */\n'
                   '  if (fabs(tNext - solverInfo->currentTime) <= '
                   '16 * DBL_EPSILON * fmax(1.0, fabs(tNext))) return 0;\n\n'
                   '  N_VSetArrayPointer(states, cvodeData->y);')
    return '#include <float.h>\n' + source.replace(needle, replacement)


def localize_cs_events(cvode: str, interface: str) -> tuple[str, str]:
    """Locate hybrid state events inside an OMC 1.27.1 CS communication step."""
    cvode_changes = {
        'omc_flag[FLAG_NO_ROOTFINDING] || isFMI': 'omc_flag[FLAG_NO_ROOTFINDING]',
        '  /* TODO: re-scale cvodeData->y to evaluate the equations */':
            '  /* Root searches evaluate interpolated states, not the last RHS states. */\n'
            '  for (long int i = 0; i < cvodeData->N; ++i)\n'
            '    data->localData[0]->realVars[i] = NV_Ith_S(y, i);',
        '  /* Error handling */\n  if ((flag == CV_SUCCESS || flag == CV_TSTOP_RETURN)':
            '  if (flag == CV_ROOT_RETURN) return 1;\n'
            '  /* Error handling */\n  if ((flag == CV_SUCCESS || flag == CV_TSTOP_RETURN)',
    }
    interface_changes = {
        '    /* adjust for time events */':
            '    zc_event = 0;\n    time_event = 0;\n    /* adjust for time events */',
        '        flag = cvode_solver_fmi_step(comp, tNext, states);':
            '        flag = cvode_solver_fmi_step(comp, tNext, states);\n'
            '        if (flag == 1) {\n'
            '          tNext = comp->solverInfo->currentTime;\n'
            '          zc_event = 1;\n'
            '          time_event = 0;\n'
            '        }',
    }
    for source, changes in ((cvode, cvode_changes), (interface, interface_changes)):
        if any(source.count(old) != 1 or (new not in old and new in source) for old, new in changes.items()):
            raise ValueError('Unsupported OMC CS source; review the event-localization workaround')
    for old, new in cvode_changes.items():
        cvode = cvode.replace(old, new)
    for old, new in interface_changes.items():
        interface = interface.replace(old, new)
    return cvode, interface


def restart_cvode_on_input_change(source: str) -> str:
    """Invalidate OMC 1.27.1's CS integrator history after an input jump."""
    start = source.find('fmi2Status fmi2SetReal(')
    end = source.find('fmi2Status fmi2SetInteger(', start)
    needle = '    if (setReal(comp, vr[i], value[i]) != fmi2OK)'
    section = source[start:end]
    if (start < 0 or end < 0 or section.count(needle) != 1 or
            'A discontinuous input invalidates' in section):
        raise ValueError('Unsupported OMC FMI source; review the input-history workaround')
    replacement = '''#if NUMBER_OF_REAL_INPUTS > 0
    /* A discontinuous input invalidates CVODE's multistep history. */
    if (comp->state == model_state_cs_step_complete &&
        comp->solverInfo->solverMethod == S_CVODE &&
        mapInputReference2InputNumber(vr[i]) >= 0 && getReal(comp, vr[i]) != value[i])
      comp->solverInfo->didEventStep = 1;
#endif
''' + needle
    return source[:start] + section.replace(needle, replacement) + source[end:]


def apply_cs_tolerance(source: str) -> str:
    """Apply FMI SetupExperiment tolerance to OMC 1.27.1's existing CVODE solver."""
    old = '  comp->toleranceDefined = toleranceDefined;\n  comp->tolerance = tolerance;'
    if source.count(old) != 1:
        raise ValueError('Unsupported OMC FMI source; review the tolerance workaround')
    new = '''  if (toleranceDefined && (!(tolerance > 0) || !isfinite(tolerance)))
    return fmi2Error;
  comp->toleranceDefined = toleranceDefined;
  comp->tolerance = toleranceDefined ? tolerance : comp->fmuData->simulationInfo->tolerance;
  if (toleranceDefined) {
    comp->fmuData->simulationInfo->tolerance = tolerance;
#ifdef WITH_SUNDIALS
    if (isCoSimulation(comp) && comp->solverInfo->solverMethod == S_CVODE) {
      CVODE_SOLVER *cvodeData = (CVODE_SOLVER*) comp->solverInfo->solverData;
      for (int i = 0; i < cvodeData->N; ++i) {
        const modelica_real nominal = getNominalFromScalarIdx(comp->fmuData->simulationInfo,
          comp->fmuData->modelData, VAR_KIND_STATE, i);
        NV_Ith_S(cvodeData->absoluteTolerance, i) = fmax(fabs(nominal), 1e-32) * tolerance;
      }
      if (CVodeSVtolerances(cvodeData->cvode_mem, tolerance, cvodeData->absoluteTolerance) != CV_SUCCESS)
        return fmi2Error;
    }
#endif
  }'''
    return source.replace(old,new)


def guard_homotopy_initial_guess(source: str) -> str:
    """Return the evaluated guess when OMC 1.27.1's nonlinear solve exits early."""
    old = '''        success = NLS_SOLVED;
        /* take the solution */
        vecCopy(homotopyData->n, homotopyData->x, nlsData->nlsx);
        /* reset continous flag */'''
    if source.count(old) != 1:
        raise ValueError('Unsupported OMC homotopy source; review the initial-guess workaround')
    return source.replace(old, old.replace('homotopyData->x,', 'homotopyData->x0,'))


def parameter_manifest(xml: Path, *, controller: bool = False) -> dict:
    root = ET.parse(xml).getroot()
    result = {'parameters': {}, 'derived': {}, 'structural': {},
              'inputs': {}, 'outputs': {}}
    result['type_definitions'] = {t.attrib['name']: {**t.attrib, 'definition': ET.tostring(t, encoding='unicode')}
                                  for t in root.findall('./TypeDefinitions/SimpleType')}
    unexpected = []
    for v in root.findall('./ModelVariables/ScalarVariable'):
        name = v.attrib['name']
        kind = v.attrib.get('causality', 'local')
        scalar = next(iter(v))
        entry = {**v.attrib, 'type': scalar.tag, **scalar.attrib}
        if name.startswith('pVehicle.'):
            if name.endswith(('springPoints', 'damperPoints', 'frontPoints', 'rearPoints')) and kind != 'parameter':
                unexpected.append(name)
                continue
            if name.startswith(DERIVED) or name in DERIVED_NAMES:
                if kind != 'calculatedParameter':
                    raise ValueError('Derived vehicle calibration lost its dependency: ' + name)
                group = 'derived'
            elif kind == 'parameter':
                group = 'parameters'
            elif scalar.tag in ('Boolean', 'Integer', 'Enumeration', 'String'):
                group = 'structural'
            else:
                unexpected.append(name)
                continue
            result[group][name] = entry
        elif kind in ('input', 'output'):
            result[kind + 's'][name] = entry
        elif kind == 'parameter' and '.' not in name:
            result['parameters'][name] = entry
        elif kind == 'calculatedParameter' and '.' not in name:
            group = 'structural' if scalar.tag in ('Boolean', 'Integer', 'Enumeration') else 'derived'
            result[group][name] = entry
    if unexpected:
        raise ValueError('Numeric vehicle calibration was frozen: ' + ', '.join(unexpected))
    required = ({'speed', 'ay', *(f'loads[{i}]' for i in range(1, 5))} if controller else
                {'steeringAngleCommand', 'acceleratorPedalCommand', 'brakePedalCommand'})
    outputs = ({'steering', 'accelerator', 'brake', 'ayTarget', 'rampEnding', 'targetReached'}
               if controller else {'accY', 'velX', 'Fz_FL'})
    if not required <= result['inputs'].keys() or not outputs <= result['outputs'].keys():
        raise ValueError('FMU is missing its public driver/measurement interface')
    if controller:
        calibration = {'mode', 'tau_max', 'finalDriveRatio', 'mechanicalBrakeTorqueLimit'}
        if not calibration <= result['parameters'].keys():
            raise ValueError('Driver calibration was frozen or lost: ' +
                             ', '.join(sorted(calibration - result['parameters'].keys())))
    return result


def check_record_inventory(flattened: str, manifest: dict) -> list[str]:
    """Catch fields optimized out entirely, which XML-only validation cannot see."""
    names = sorted(set(re.findall(
        r'^\s*(?:final\s+)?parameter\s+(?:Real|Integer|Boolean|String|enumeration\([^\n]*?\))\s+'
        r'(pVehicle\.[^\s(=;]+)', flattened, re.MULTILINE)))
    if not names:
        raise ValueError('No VehicleDefn parameters found in flattened source')
    present = set().union(*(manifest[group] for group in ('parameters', 'derived', 'structural')))
    missing = sorted(set(names)-present)
    if missing:
        raise ValueError('VehicleDefn fields absent from artifact: ' + ', '.join(missing))
    return names


def validate_record_inventory(name: str, wrapper: Path, work: Path, manifest: dict,
                              destination: Path) -> float:
    started = time.perf_counter()
    flattened = work / 'calibration-inventory.mo'
    script = work / 'calibration-inventory.mos'
    script.write_text(f'''loadModel(Modelica, {{{json.dumps(ARCHITECTURES["libraries"]["Modelica"])}}});
loadModel(VehicleInterfaces, {{{json.dumps(ARCHITECTURES["libraries"]["VehicleInterfaces"])}}});
loadFile({json.dumps(str(ROOT / 'BobLib/package.mo'))});
loadFile({json.dumps(str(wrapper))});
writeFile({json.dumps(str(flattened))}, instantiateModel({name}));
getErrorString();
''')
    run(['omc', str(script)], work, destination / 'calibration-inventory.log')
    manifest['source_parameter_inventory'] = check_record_inventory(flattened.read_text(), manifest)
    return time.perf_counter()-started


def compiler_timings(log: str, *, template_phase: str = 'buildModelFMU: Generate the FMI files') -> dict:
    """OMC execstat clocks are cumulative within buildModelFMU."""
    events = [(name, float(elapsed), float(total)) for name, elapsed, total in re.findall(
        r'Performance of (.+?): time ([0-9.eE+-]+)/([0-9.eE+-]+),', log)]
    cumulative = {name: total for name, _, total in events}
    names = ['FrontEnd', 'Backend phase and start with SimCode phase', 'SimCode',
             template_phase]
    if not all(name in cumulative for name in names):
        raise ValueError('Missing OMC compiler phase clocks')
    values = [cumulative[name] for name in names]
    if values != sorted(values):
        raise ValueError('Nonmonotonic OMC compiler phase clocks')
    phases = dict(zip(['frontend_seconds', 'backend_seconds', 'simcode_seconds',
                       'templates_seconds'], [values[0]] +
                      [values[i] - values[i - 1] for i in range(1, 4)], strict=True))
    phases['largest_compiler_passes'] = [{'pass': n, 'seconds': t}
                                        for n, t, _ in sorted(events, key=lambda e: e[1], reverse=True)[:10]]
    return phases


def run(command: list[str], cwd: Path, log: Path) -> None:
    with log.open('w') as stream:
        subprocess.run(command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT,
                       check=True)


def build(front: str, rear: str, output: Path, jobs: int, *, rotor_effects: bool = False,
          differential: str = DEFAULTS['differential'], front_brakes: str = DEFAULTS['front_brakes'],
          rear_brakes: str = DEFAULTS['rear_brakes']) -> Path:
    name, source = vehicle_model(front, rear, rotor_effects=rotor_effects, differential=differential,
                                 front_brakes=front_brakes, rear_brakes=rear_brakes)
    return build_source(name, source, output, jobs,
                        {'front': front, 'rear': rear, 'rotor_effects': rotor_effects,
                         'differential': differential, 'front_brakes': front_brakes,
                         'rear_brakes': rear_brakes, 'fixed_models': ARCHITECTURES['fixed_models']})


def build_driver(output: Path, jobs: int) -> Path:
    name = 'BobLibDriver'
    source = f'model {name}\n  extends BobLib.Experiments.Standards.ManeuverFMI;\nend {name};\n'
    return build_source(name, source, output, jobs, {'kind': 'driver'}, controller=True)


def build_source(name: str, source: str, output: Path, jobs: int,
                 configuration: dict, *, controller: bool = False) -> Path:
    started = time.perf_counter()
    source_identity = source_fingerprint(ROOT)
    phases = {}
    destination = output / name
    destination.mkdir(parents=True, exist_ok=True)
    with contextlib.nullcontext(tempfile.mkdtemp(prefix='boblib-fmu-')) as directory:
        work = Path(directory)
        model = work / f'{name}.mo'
        model.write_text(source)
        script = work / 'export.mos'
        # Combined export avoids OMC's CS-only output-derivative index reduction.
        script.write_text(f'''loadModel(Modelica, {{{json.dumps(ARCHITECTURES["libraries"]["Modelica"])}}});
loadModel(VehicleInterfaces, {{{json.dumps(ARCHITECTURES["libraries"]["VehicleInterfaces"])}}});
loadFile({json.dumps(str(ROOT / 'BobLib/package.mo'))});
loadFile({json.dumps(str(model))});
setCommandLineOptions({json.dumps(FLAGS.replace("disableStartCalc", "disableStartCalc,execstat"))});
buildModelFMU({name},version="2.0",fmuType="me_cs",fileNamePrefix="{name}",platforms={{""}});
getErrorString();
''')
        shutil.copy2(script, destination / 'export.mos')
        (destination / f'{name}.mo').write_text(source)
        before = time.perf_counter()
        run(['omc', str(script)], work, destination / 'translation.log')
        phases['omc_process_seconds'] = time.perf_counter() - before
        phases.update(compiler_timings((destination / 'translation.log').read_text()))
        before = time.perf_counter()
        archive = work / f'{name}.fmu'
        unpacked = work / 'unpacked'
        if archive.is_file():
            with zipfile.ZipFile(archive) as z:
                z.extractall(unpacked)
        else:
            # OMC 1.27.1 leaves source-only exports unpacked.
            candidates = [p for p in work.glob('*.fmutmp')
                          if (p / 'modelDescription.xml').is_file()]
            if len(candidates) != 1:
                raise RuntimeError(f'OMC produced no FMU; see {destination / "translation.log"}')
            shutil.move(candidates[0], unpacked)
        sources = unpacked / 'sources'
        init = sources / f'{name}_init_fmu.c'
        init.write_text(split_initializer(init.read_text(), name))
        cvode = sources / 'simulation/solver/cvode_solver.c'
        cvode.write_text(guard_cvode_events(cvode.read_text()))
        interface = sources / 'fmi-export/fmu2_model_interface.c'
        interface.write_text(apply_cs_tolerance(restart_cvode_on_input_change(interface.read_text())))
        cvode_source, interface_source = localize_cs_events(cvode.read_text(), interface.read_text())
        cvode.write_text(cvode_source)
        interface.write_text(interface_source)
        homotopy = sources / 'simulation/solver/nonlinearSolverHomotopy.c'
        if homotopy.is_file():
            homotopy.write_text(guard_homotopy_initial_guess(homotopy.read_text()))
        cmake_source = sources / 'CMakeLists.txt'
        with cmake_source.open('a') as stream:
            stream.write(f'\nset_source_files_properties({name}_init_fmu.c '
                         'PROPERTIES COMPILE_OPTIONS "-O0")\n')
        manifest = parameter_manifest(unpacked / 'modelDescription.xml', controller=controller)
        if not controller:
            phases['calibration_inventory_seconds'] = validate_record_inventory(
                name, model, work, manifest, destination)
        manifest.update({'model': name, **configuration,
                         'source': source_identity,
                         'omc': subprocess.check_output(['omc', '--version'], text=True).strip(),
                         'host': platform.platform(), 'machine': platform.machine(),
                         'compiler_flags': FLAGS,
                         'c_optimization': {'physics': '-Os', 'metadata': '-O0'},
                         'table_capacities': {} if controller else {'curve_points': CURVE_CAPACITY,
                                              'aero_points_per_axis': AERO_CAPACITY},
                         'workarounds': ['split_metadata_initializer', 'cvode_event_roundoff_guard',
                                         'cvode_input_history_reset', 'cs_setup_tolerance', 'cs_event_localization'] +
                                        (['homotopy_initial_guess_return'] if homotopy.is_file() else []),
                         'parameter_lifecycle': 'Set before initialization; array dimensions are fixed.'})
        phases['source_preparation_seconds'] = (time.perf_counter() - before
                                                 - phases.get('calibration_inventory_seconds', 0))
        cmake = work / 'cmake'
        before = time.perf_counter()
        run(['cmake', '-S', str(sources), '-B', str(cmake),
             '-DCMAKE_C_COMPILER=clang', '-DCMAKE_C_FLAGS_RELEASE=-Os -DNDEBUG'],
            work, destination / 'configure.log')
        phases['cmake_configure_seconds'] = time.perf_counter() - before
        before = time.perf_counter()
        run(['cmake', '--build', str(cmake), '--parallel', str(jobs), '--target', 'install'],
            work, destination / 'compile.log')
        phases['compile_link_install_seconds'] = time.perf_counter() - before
        before = time.perf_counter()
        manifest['binaries'] = [str(p.relative_to(unpacked))
                               for p in (unpacked / 'binaries').rglob('*') if p.is_file()]
        if not manifest['binaries']:
            raise RuntimeError('Build produced no FMI binary')
        if source_fingerprint(ROOT) != source_identity:
            raise RuntimeError('BobLib sources changed during export; rebuild from stable sources')
        (unpacked / 'resources/boblib-build.json').write_text(
            json.dumps(manifest, indent=2) + '\n')
        final = destination / f'{name}.fmu'
        temporary = destination / f'{name}.fmu.tmp'
        with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED) as z:
            for p in sorted(unpacked.rglob('*')):
                if p.is_file():
                    z.write(p, p.relative_to(unpacked))
        temporary.replace(final)
        manifest['sha256'] = hashlib.sha256(final.read_bytes()).hexdigest()
        (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        phases['package_seconds'] = time.perf_counter() - before
        shutil.rmtree(work)
        phases['total_build_seconds'] = time.perf_counter() - started
        phases['compile_jobs'] = jobs
        (destination / 'build-timings.json').write_text(json.dumps(phases, indent=2) + '\n')
        return final


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front', choices=AXLES, default=DEFAULTS['front'])
    parser.add_argument('--rear', choices=AXLES, default=DEFAULTS['rear'])
    kind = parser.add_mutually_exclusive_group()
    kind.add_argument('--all', action='store_true', help='Build all nine suspension combinations')
    kind.add_argument('--driver', action='store_true', help='Build the standard maneuver driver FMU')
    parser.add_argument('--differential', choices=ARCHITECTURES['differentials'], default=DEFAULTS['differential'])
    parser.add_argument('--front-brakes', choices=ARCHITECTURES['front_brakes'], default=DEFAULTS['front_brakes'])
    parser.add_argument('--rear-brakes', choices=ARCHITECTURES['rear_brakes'], default=DEFAULTS['rear_brakes'])
    parser.add_argument('--rotor-effects', action='store_true',
                        help='Include rotor gyroscopic and carrier-acceleration effects (supports are always active)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    if args.driver:
        if args.rotor_effects:
            parser.error('--rotor-effects applies to the vehicle plant')
        print(build_driver(args.output.resolve(), args.jobs), flush=True)
        return
    pairs = [(f, r) for f in AXLES for r in AXLES] if args.all else [(args.front, args.rear)]
    for front, rear in pairs:
        print(build(front, rear, args.output.resolve(), args.jobs,
                    rotor_effects=args.rotor_effects, differential=args.differential,
                    front_brakes=args.front_brakes, rear_brakes=args.rear_brakes), flush=True)


if __name__ == '__main__':
    main()
