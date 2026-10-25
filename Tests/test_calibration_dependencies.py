"""The same native/FMU artifacts must recalculate record dependencies per run."""
import csv
import importlib.util
import json
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('dependency_export', ROOT / 'Tools/export_fmu.py')
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


def test_record_dependencies_recalculate_in_native_and_fmu(tmp_path):
    if shutil.which('omc') is None:
        pytest.skip('OpenModelica is required')
    runtime = pytest.importorskip('fmpy')
    import numpy as np
    source = '''model Dependencies
  parameter BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle;
  inner parameter Boolean headless=true;
  inner parameter Real linkDiameter=0.02;
  inner parameter Real jointDiameter=0.03;
  inner Modelica.Mechanics.MultiBody.World world(g=0,enableAnimation=false);
  Modelica.Mechanics.MultiBody.Parts.Fixed anchor;
  BobLib.Chassis.Body.FrameCompX frame(pFrame=pVehicle.chassis.frame);
  output Real capacity=pVehicle.battery.Q_cell;
  output Real resistance=pVehicle.battery.R0_dis_cell_table[1];
  output Real efficiency=pVehicle.inverter.eta_mot_table[1];
  output Real ratio=pVehicle.controller.finalDriveRatio;
  output Real brakeRatio=pVehicle.brakes.differentialBrakeRatio;
  output Real brakeTorque=pVehicle.brakes.maxTorque;
  output Real frameReference=pVehicle.chassis.frame.frontReference[1];
  output Real damping=pVehicle.driveline.halfshaft.damping;
  output Real frontDamping=pVehicle.driveline.frontBrakeHalfshaft.damping;
  output Real frameDamping=pVehicle.chassis.frame.torsionalDamping;
  output Real inertia=pVehicle.chassis.frame.mass.inertia[1,1];
  output Real mass=pVehicle.chassis.frame.mass.m;
  output Real wheelTranslationDamping=pVehicle.chassis.frontAxle.tire.wheel.compliance.translationDamping[1];
  output Real wheelRotationDamping=pVehicle.chassis.rearAxle.tire.wheel.compliance.rotationDamping[3];
  output Real clock=time;
end Dependencies;
'''
    mass_outputs = ''.join(f'  output Real cm{i}=pVehicle.chassis.frame.mass.rCM[{i}];\n'
                           for i in range(1, 4))
    mass_outputs += ''.join(f'  output Real I{i}{j}=pVehicle.chassis.frame.mass.inertia[{i},{j}];\n'
                            for i in range(1, 4) for j in range(1, 4))
    mass_outputs += ''.join(
        f'  output Real frameCG{i}=frame.pFrame.mass.rCM[{i}];\n'
        f'  output Real frameOffset{i}=frame.midToCG.r[{i}];\n'
        for i in range(1, 4))
    source = source.replace('end Dependencies;', mass_outputs +
                            'equation\n  connect(anchor.frame_b,frame.frontFrame);\nend Dependencies;')
    (tmp_path / 'Dependencies.mo').write_text(source)
    script = f'''loadModel(Modelica, {{"4.1.0"}});
loadModel(VehicleInterfaces, {{"2.0.2"}});
loadFile({json.dumps(str(ROOT / 'BobLib/package.mo'))});
loadFile("Dependencies.mo");
setCommandLineOptions({json.dumps(exporter.FLAGS)});
buildModel(Dependencies, outputFormat="csv");
getErrorString();
buildModelFMU(Dependencies, version="2.0", fmuType="me_cs");
getErrorString();
'''
    (tmp_path / 'build.mos').write_text(script)
    process = subprocess.run(['omc', 'build.mos'], cwd=tmp_path, capture_output=True,
                             text=True, timeout=300)
    (tmp_path / 'build.log').write_text(process.stdout + process.stderr)
    assert process.returncode == 0 and 'Error:' not in process.stdout, process.stdout
    xml = ET.parse(tmp_path / 'Dependencies_init.xml')
    native_vars = {v.get('name'): v for v in xml.findall('./ModelVariables/ScalarVariable')}
    md = runtime.read_model_description(str(tmp_path / 'Dependencies.fmu'), validate=True)
    fmi_vars = {v.name: v for v in md.modelVariables}
    wheel_derived = {f'pVehicle.chassis.{axle}.tire.wheel.compliance.{kind}Damping[{i}]'
                     for axle in ('frontAxle', 'rearAxle')
                     for kind in ('translation', 'rotation') for i in range(1, 4)}
    for name in exporter.DERIVED_NAMES | wheel_derived:
        assert native_vars[name].get('causality') == 'calculatedParameter', name
        assert native_vars[name].get('isValueChangeable') == 'false', name
        assert fmi_vars[name].causality == 'calculatedParameter', name
    values = {
        'pVehicle.chassis.frontAxle.tire.wheel.compliance.translationStiffness[1]': 4e7,
        'pVehicle.chassis.rearAxle.tire.wheel.compliance.rotationStiffness[3]': 4e5,
        'pVehicle.battery.E_cell': 30000., 'pVehicle.battery.V_cell_nominal': 4.,
        'pVehicle.battery.R_cell': .004, 'pVehicle.inverter.eta_mot': .9,
        'pVehicle.transmission.gearRatio': 2., 'pVehicle.driveline.inputRatio': 3.,
        'pVehicle.controller.mechanicalBrakeTorqueLimit': 1700.,
        'pVehicle.chassis.frontAxle.geometry.wheelCenter[1]': .02,
        'pVehicle.driveline.halfshaft.stiffness': 25000.,
        'pVehicle.driveline.halfshaft.inertia': .04,
        'pVehicle.driveline.frontBrakeHalfshaft.stiffness': 10000.,
        'pVehicle.driveline.frontBrakeHalfshaft.inertia': .05,
        'pVehicle.chassis.frame.torsionalStiffness': 200000.,
        'pVehicle.chassis.frame.driverMass.m': 75.770912,
        'pVehicle.chassis.frame.driverMass.rCM[2]': .04,
    }
    outputs = re.findall(r'output Real (\w+)=', source)
    results = []
    for index, settings in enumerate(({}, values, {})):
        for name in settings:
            assert native_vars[name].get('isValueChangeable') == 'true'
            assert fmi_vars[name].causality == 'parameter'
        override = tmp_path / f'override-{index}.txt'
        override.write_text('\n'.join(f'{key}={value}' for key, value in settings.items()) + '\n')
        args = [str(tmp_path / 'Dependencies'), '-stopTime=0.01',
                f'-r=native-{index}.csv', '-output=' + ','.join(outputs)]
        if settings:
            args.append(f'-overrideFile={override}')
        run = subprocess.run(args, cwd=tmp_path, text=True, capture_output=True, timeout=30)
        assert run.returncode == 0 and 'simulation finished successfully' in run.stdout, run.stdout
        with (tmp_path / f'native-{index}.csv').open() as stream:
            row = list(csv.DictReader(stream))[-1]
        native = {}
        for name in outputs:
            match = re.search(r'\b' + name + r'\s*=\s*([-+0-9.eE]+)', run.stdout)
            native[name] = float(row[name]) if name in row else float(match[1])
        result = runtime.simulate_fmu(str(tmp_path / 'Dependencies.fmu'),
                                      fmi_type='CoSimulation', stop_time=.01,
                                      start_values=settings, output=outputs)
        actual = {name: float(result[name][-1]) for name in outputs}
        assert actual == pytest.approx(native, rel=1e-10, abs=1e-10)
        def calibrated(name, values):
            return values.get(name, float(fmi_vars[name].start))

        masses, centers, inertias = [], [], []
        for owner in ('baseMass', 'driverMass'):
            prefix = f'pVehicle.chassis.frame.{owner}'
            masses.append(calibrated(prefix + '.m', settings))
            centers.append(np.array([calibrated(f'{prefix}.rCM[{i}]', settings) for i in range(1, 4)]))
            inertias.append(np.array([[calibrated(f'{prefix}.inertia[{i},{j}]', settings)
                                       for j in range(1, 4)] for i in range(1, 4)]))
        center = sum(m*c for m, c in zip(masses, centers, strict=True))/sum(masses)
        tensor = np.zeros((3, 3))
        for mass, position, inertia_tensor in zip(masses, centers, inertias, strict=True):
            delta = position - center
            tensor += inertia_tensor + mass*(np.dot(delta, delta)*np.eye(3) - np.outer(delta, delta))
        assert actual['mass'] == pytest.approx(sum(masses))
        np.testing.assert_allclose([actual[f'cm{i}'] for i in range(1, 4)], center, atol=1e-12)
        np.testing.assert_allclose([actual[f'frameCG{i}'] for i in range(1, 4)], center, atol=1e-12)
        midpoint = sum(np.array([
            calibrated(f'pVehicle.chassis.{axle}.geometry.wheelCenter[1]', settings),
            0,
            calibrated(f'pVehicle.chassis.{axle}.geometry.wheelCenter[3]', settings)])
            for axle in ('frontAxle', 'rearAxle'))/2
        np.testing.assert_allclose([actual[f'frameOffset{i}'] for i in range(1, 4)],
                                   center-midpoint, atol=1e-12)
        np.testing.assert_allclose([[actual[f'I{i}{j}'] for j in range(1, 4)]
                                    for i in range(1, 4)], tensor, atol=1e-12)
        results.append(actual)
    expected = dict(capacity=7500., resistance=.0064, efficiency=.78, ratio=6.,
                    brakeRatio=3., brakeTorque=1700., frameReference=.02,
                    damping=2*(25000*.04)**.5, frontDamping=2*(10000*.05)**.5,
                    wheelTranslationDamping=1.4*(4e7*10)**.5,
                    wheelRotationDamping=1.4*(4e5*.1)**.5)
    for name, value in expected.items():
        assert results[1][name] == pytest.approx(value)
    assert results[1]['mass'] - results[0]['mass'] == pytest.approx(10.)
    assert results[1]['frameDamping'] == pytest.approx(2*(200000*results[1]['inertia'])**.5)
    assert results[0] == pytest.approx(results[2])
    for name in expected:
        assert results[0][name] != results[1][name], name
