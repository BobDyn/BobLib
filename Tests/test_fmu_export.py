"""Export boundary and safeguards for reusable vehicle FMUs."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('export_fmu', ROOT / 'Tools/export_fmu.py')
fmu = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fmu)


def test_source_identity_includes_uncommitted_models_and_exporter(tmp_path):
    (tmp_path / 'BobLib').mkdir()
    (tmp_path / 'Tools').mkdir()
    model = tmp_path / 'BobLib/package.mo'
    model.write_text('package BobLib end BobLib;')
    exporter = tmp_path / 'Tools/export_fmu.py'
    exporter.write_text('original runtime patches')
    original = fmu.source_fingerprint(tmp_path)
    assert fmu.source_fingerprint(tmp_path) == original
    model.write_text('package BobLib parameter Real x=1; end BobLib;')
    changed = fmu.source_fingerprint(tmp_path)
    assert changed['sha256'] != original['sha256']
    exporter.write_text('new runtime patches')
    patched = fmu.source_fingerprint(tmp_path)
    assert patched['sha256'] != changed['sha256']
    (tmp_path / 'BobLib/package.order').write_text('Example\n')
    assert fmu.source_fingerprint(tmp_path)['sha256'] != patched['sha256']
    assert set(original['files']) == {'BobLib/package.mo', 'Tools/export_fmu.py'}


def test_all_architectures_reference_existing_classes():
    names = set()
    for front in fmu.AXLES:
        for rear in fmu.AXLES:
            name, source = fmu.vehicle_model(front, rear)
            names.add(name)
            fr, _ = fmu.AXLES[front]
            rr, _ = fmu.AXLES[rear]
            assert (ROOT / 'BobLib/Records/VehicleDefn' /
                    f'EVBatInvMotDiff_{fr}_{rr}Record.mo').is_file()
            assert 'initialLongitudinalVelocity=initialVel' in source
            assert 'final flatRoadFastPath=true' in source
            assert 'pChassis = pVehicle.chassis' in source
            assert f'redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_{fr}_{rr}Record.ChassisRecord' in source
            assert 'damping=' not in source
            assert 'springPoints=size(defaultVehicle.chassis.frontAxle.actuation.springTable, 1)' in source
            assert f'zeros({fmu.CURVE_CAPACITY}-size(' in source
            assert f', {fmu.AERO_CAPACITY}))' in source
    assert len(names) == 9


def test_rotor_effects_have_distinct_artifact_and_fixed_equations():
    basic, basic_source = fmu.vehicle_model('stabar', 'bellcrank')
    with_rotors, rotor_source = fmu.vehicle_model('stabar', 'bellcrank', rotor_effects=True)
    assert with_rotors == basic + '_RotorEffects'
    assert 'final includeRotorEffects=false' in basic_source
    assert 'final includeRotorEffects=true' in rotor_source


def test_initializer_preserves_assignments_and_rejects_control_flow():
    signature = 'void Vehicle_read_input_fmu(MODEL_DATA* modelData)'
    lines = [f'  modelData->x[{i}] = {i};\n' for i in range(2001)]
    source = '#include "simulation_data.h"\n' + signature + '\n{\n' + ''.join(lines) + '}\n'
    result = fmu.split_initializer(source, 'Vehicle')
    assert fmu.split_initializer(source.rstrip('\n'), 'Vehicle') == result
    assert [line for line in result.splitlines(keepends=True)
            if line.lstrip().startswith('modelData->')] == lines
    assert 'metadata_chunk_1(modelData);' in result
    with pytest.raises(ValueError):
        fmu.split_initializer(source.replace(lines[0], '  if (1) {\n'), 'Vehicle')


def description(tmp_path, *, frozen=False):
    root = ET.Element('fmiModelDescription')
    variables = ET.SubElement(root, 'ModelVariables')
    entries = [('pVehicle.chassis.frontAxle.geometry.upperFore_i[1]', 'calculatedParameter' if frozen else 'parameter'),
               ('pVehicle.chassis.frame.mass.m', 'calculatedParameter')]
    entries += [(name, 'input') for name in ('steeringAngleCommand', 'acceleratorPedalCommand', 'brakePedalCommand')]
    entries += [(name, 'output') for name in ('accY', 'velX', 'Fz_FL')]
    for name, causality in entries:
        v = ET.SubElement(variables, 'ScalarVariable', name=name, causality=causality)
        ET.SubElement(v, 'Real', start='0')
    path = tmp_path / 'modelDescription.xml'
    ET.ElementTree(root).write(path)
    return path


def test_manifest_rejects_silently_frozen_geometry(tmp_path):
    with pytest.raises(ValueError, match='Numeric vehicle calibration was frozen'):
        fmu.parameter_manifest(description(tmp_path, frozen=True))
    manifest = fmu.parameter_manifest(description(tmp_path))
    assert 'pVehicle.chassis.frontAxle.geometry.upperFore_i[1]' in manifest['parameters']
    assert 'pVehicle.chassis.frame.mass.m' in manifest['derived']



@pytest.mark.parametrize("causality", ["parameter", "local"])
def test_manifest_rejects_lost_derived_mass_dependency(tmp_path, causality):
    xml = description(tmp_path)
    root = ET.parse(xml)
    for variable in root.findall('./ModelVariables/ScalarVariable'):
        if variable.get('name') == 'pVehicle.chassis.frame.mass.m':
            variable.set('causality', causality)
    root.write(xml)
    with pytest.raises(ValueError, match='Derived vehicle calibration'):
        fmu.parameter_manifest(xml)

def test_runtime_patch_fails_closed_on_unrecognized_compiler_source():
    with pytest.raises(ValueError, match='Unsupported OMC CVODE source'):
        fmu.guard_cvode_events('different runtime implementation')


def test_homotopy_early_success_returns_the_evaluated_guess():
    branch = '''        success = NLS_SOLVED;
        /* take the solution */
        vecCopy(homotopyData->n, homotopyData->x, nlsData->nlsx);
        /* reset continous flag */'''
    other = '\nvecCopy(homotopyData->n, homotopyData->x, nlsData->nlsx);\n'
    patched = fmu.guard_homotopy_initial_guess(branch + other)
    assert patched == branch.replace('homotopyData->x,', 'homotopyData->x0,') + other
    for unsupported in ('different source', branch + branch, patched):
        with pytest.raises(ValueError, match='Unsupported OMC homotopy source'):
            fmu.guard_homotopy_initial_guess(unsupported)


def test_active_counts_cannot_silently_become_structural(tmp_path):
    xml = description(tmp_path)
    root = ET.parse(xml)
    v = ET.SubElement(root.find('ModelVariables'), 'ScalarVariable',
                      name='pVehicle.chassis.frontAxle.actuation.springPoints', causality='calculatedParameter')
    ET.SubElement(v, 'Integer', start='2')
    root.write(xml)
    with pytest.raises(ValueError, match='springPoints'):
        fmu.parameter_manifest(xml)


def test_parameter_zero_crossing_keeps_linear_system_solvable(tmp_path):
    """A valid calibration can make a chosen symbolic divisor zero."""
    import shutil
    import subprocess

    import numpy as np

    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is required')
    runtime = pytest.importorskip('fmpy')
    (tmp_path / 'Calibration.mo').write_text('''model Calibration
  parameter Real a=1 annotation(Evaluate=false);
  parameter Real b=0 annotation(Evaluate=false);
  output Real x;
  output Real y;
  Real z(start=0, fixed=true);
equation
  a*x+b*y=1;
  b*x+a*y=2;
  der(z)=x;
end Calibration;
''')
    script = tmp_path / 'build.mos'
    script.write_text(f'''loadFile("Calibration.mo");
setCommandLineOptions("{fmu.FLAGS}");
buildModelFMU(Calibration,version="2.0",fmuType="me_cs");
getErrorString();
''')
    result = subprocess.run(['omc', str(script)], cwd=tmp_path, capture_output=True,
                            text=True, timeout=120)
    archive = tmp_path / 'Calibration.fmu'
    assert result.returncode == 0 and archive.is_file(), result.stdout + result.stderr
    for a, b, expected in [(1., 0., [1., 2.]), (0., 1., [2., 1.])]:
        result = runtime.simulate_fmu(str(archive), fmi_type='CoSimulation',
                                      stop_time=.01, start_values={'a': a, 'b': b},
                                      output=['x', 'y'])
        np.testing.assert_allclose([result['x'][-1], result['y'][-1]], expected)


def test_compiler_phase_timings_do_not_double_count_cumulative_clocks():
    log = '\n'.join(f'Notification: Performance of {name}: time {elapsed}/{total}, allocations: 0'
                    for name, elapsed, total in [
                        ('FrontEnd', .1, 2),
                        ('Backend phase and start with SimCode phase', .2, 12),
                        ('SimCode', .01, 15),
                        ('buildModelFMU: Generate the FMI files', 4, 19)])
    result = fmu.compiler_timings(log)
    assert [result[k] for k in ('frontend_seconds', 'backend_seconds', 'simcode_seconds', 'templates_seconds')] == [2, 10, 3, 4]
    with pytest.raises(ValueError, match='Missing OMC'):
        fmu.compiler_timings('')


def test_input_change_restarts_only_active_cs_cvode(tmp_path):
    """Execute the patched setter: unchanged values and ME must retain history."""
    import shutil
    import subprocess

    cc = shutil.which('cc')
    if cc is None:
        pytest.skip('C compiler is required')
    source = '''
#include <assert.h>
#define NUMBER_OF_REAL_INPUTS 1
#define model_state_cs_step_complete 4
#define S_CVODE 8
#define fmi2OK 0
#define fmi2Error 3
typedef int fmi2Status;
typedef struct { int solverMethod; int didEventStep; } Solver;
typedef struct { int state; Solver *solverInfo; double values[2]; } ModelInstance;
int mapInputReference2InputNumber(int vr) { return vr == 0 ? 0 : -1; }
double getReal(ModelInstance *comp, int vr) { return comp->values[vr]; }
int setReal(ModelInstance *comp, int vr, double v) { comp->values[vr] = v; return 0; }
fmi2Status fmi2SetReal(ModelInstance *comp, int vr[], double value[]) {
  int i=0;
    if (setReal(comp, vr[i], value[i]) != fmi2OK)
      return fmi2Error;
  return fmi2OK;
}
fmi2Status fmi2SetInteger(void) { return 0; }
int main(void) {
  Solver solver = {S_CVODE, 0};
  ModelInstance comp = {model_state_cs_step_complete, &solver, {0, 0}};
  int vr[] = {0}; double value[] = {0};
  fmi2SetReal(&comp, vr, value); assert(!solver.didEventStep);
  value[0]=1; fmi2SetReal(&comp, vr, value); assert(solver.didEventStep);
  solver.didEventStep=0;
  fmi2SetReal(&comp, vr, value); assert(!solver.didEventStep);
  vr[0]=1; fmi2SetReal(&comp, vr, value); assert(!solver.didEventStep);
  vr[0]=0; value[0]=2; comp.state=0;
  fmi2SetReal(&comp, vr, value); assert(!solver.didEventStep);
  comp.state=model_state_cs_step_complete; solver.solverMethod=0; value[0]=3;
  fmi2SetReal(&comp, vr, value); assert(!solver.didEventStep);
  return 0;
}
'''
    patched = fmu.restart_cvode_on_input_change(source)
    with pytest.raises(ValueError, match='Unsupported OMC FMI source'):
        fmu.restart_cvode_on_input_change('unexpected source')
    with pytest.raises(ValueError, match='Unsupported OMC FMI source'):
        fmu.restart_cvode_on_input_change(patched)
    path = tmp_path / 'setter.c'
    path.write_text(patched)
    binary = tmp_path / 'setter'
    subprocess.run([cc, str(path), '-o', str(binary)], check=True, capture_output=True)
    subprocess.run([str(binary)], check=True)


def test_driver_manifest_requires_complete_scalar_contract(tmp_path):
    root=ET.Element('fmiModelDescription')
    variables=ET.SubElement(root,'ModelVariables')
    for causality,names in [('input',['speed','ay']+[f'loads[{i}]' for i in range(1,5)]),
                            ('parameter',['mode','tau_max','finalDriveRatio','mechanicalBrakeTorqueLimit']),
                            ('output',['steering','accelerator','brake','ayTarget','rampEnding','targetReached'])]:
        for name in names:
            v=ET.SubElement(variables,'ScalarVariable',name=name,causality=causality)
            ET.SubElement(v,'Boolean' if name in ('rampEnding','targetReached') else 'Real')
    xml=tmp_path/'driver.xml'
    ET.ElementTree(root).write(xml)
    assert len(fmu.parameter_manifest(xml,controller=True)['inputs'])==6
    capacity = next(v for v in variables if v.attrib['name'] == 'tau_max')
    capacity.set('causality', 'calculatedParameter')
    ET.ElementTree(root).write(xml)
    with pytest.raises(ValueError, match='Driver calibration was frozen or lost: tau_max'):
        fmu.parameter_manifest(xml, controller=True)
    capacity.set('causality', 'parameter')
    variables.remove(variables[-1])
    ET.ElementTree(root).write(xml)
    with pytest.raises(ValueError,match='public driver/measurement interface'):
        fmu.parameter_manifest(xml,controller=True)


def test_explicit_architecture_choices_are_fixed_and_named():
    name, source = fmu.vehicle_model('direct', 'bellcrank', differential='TorqueBiasing',
                                     front_brakes='Inboard', rear_brakes='DifferentialInput')
    assert name.endswith('_TorqueBiasing_FrontInboard_RearDifferentialInput')
    assert 'final differentialArchitecture=BobLib.Drivelines.Types.DifferentialArchitecture.TorqueBiasing' in source
    assert 'final frontBrakePlacement=BobLib.Brakes.Types.Placement.Inboard' in source
    assert 'final rearBrakePlacement=BobLib.Brakes.Types.Placement.DifferentialInput' in source
    with pytest.raises(ValueError, match='front_brakes'):
        fmu.vehicle_model('direct', 'direct', front_brakes='DifferentialInput')
    with pytest.raises(ValueError, match='differentials'):
        fmu.vehicle_model('direct', 'direct', differential='typo')


def test_architecture_manifest_is_part_of_source_identity(tmp_path):
    (tmp_path/'BobLib').mkdir()
    (tmp_path/'Tools').mkdir()
    p=tmp_path/'Tools/architectures.json'
    p.write_text('{}')
    before=fmu.source_fingerprint(tmp_path)
    p.write_text('{"schema_version":1}')
    assert fmu.source_fingerprint(tmp_path)['sha256'] != before['sha256']


def test_source_inventory_rejects_fields_missing_from_xml():
    flattened = '''class Plant
  parameter Real pVehicle.pickup[1](unit="m") = 1;
  final parameter enumeration(Open, Locked) pVehicle.architecture;
  parameter Integer pVehicle.curvePoints = 2;
  parameter Real unrelated = 0;
end Plant;'''
    manifest = {'parameters': {'pVehicle.pickup[1]': {}}, 'derived': {},
                'structural': {'pVehicle.architecture': {}}}
    with pytest.raises(ValueError, match='pVehicle.curvePoints'):
        fmu.check_record_inventory(flattened, manifest)
    manifest['parameters']['pVehicle.curvePoints'] = {}
    assert fmu.check_record_inventory(flattened, manifest) == [
        'pVehicle.architecture', 'pVehicle.curvePoints', 'pVehicle.pickup[1]']
    with pytest.raises(ValueError, match='No VehicleDefn'):
        fmu.check_record_inventory('', manifest)


def test_cs_tolerance_patch_rejects_changed_or_already_patched_source():
    original = '  comp->toleranceDefined = toleranceDefined;\n  comp->tolerance = tolerance;'
    patched = fmu.apply_cs_tolerance(original)
    assert 'CVodeSVtolerances' in patched
    for source in ('', original + original, patched):
        with pytest.raises(ValueError, match='tolerance workaround'):
            fmu.apply_cs_tolerance(source)


@pytest.mark.parametrize('name', sorted(fmu.DERIVED_NAMES))
@pytest.mark.parametrize('causality', ['calculatedParameter', 'parameter', 'local'])
def test_dependent_calibration_is_never_editable(tmp_path, name, causality):
    xml = description(tmp_path)
    root = ET.parse(xml)
    variable = ET.SubElement(root.find('./ModelVariables'), 'ScalarVariable',
                             name=name, causality=causality)
    ET.SubElement(variable, 'Real')
    root.write(xml)
    if causality != 'calculatedParameter':
        with pytest.raises(ValueError, match='lost its dependency'):
            fmu.parameter_manifest(xml)
    else:
        manifest = fmu.parameter_manifest(xml)
        assert name not in manifest['parameters']
        assert name in manifest['derived']
