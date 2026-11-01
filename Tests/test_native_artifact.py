"""Native validation runs the packaged artifact with per-run calibration and settings."""
import importlib.util
from pathlib import Path

import pytest

np = pytest.importorskip('numpy')
pytest.importorskip('fmpy')
SPEC = importlib.util.spec_from_file_location('verify_fmu', Path(__file__).resolve().parents[1] / 'Tools/verify_fmu.py')
verify = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify)


def test_native_settings_and_array_parameter_round_trip(tmp_path):
    executable = tmp_path / 'Vehicle'
    executable.write_text('''#!/usr/bin/env python3
import sys, pathlib, xml.etree.ElementTree as ET
args=dict(x[1:].split('=',1) for x in sys.argv[1:])
settings=ET.parse(args['f']).find('DefaultExperiment')
assert settings.get('stopTime')=='0.2'
assert settings.get('stepSize')=='0.002'
assert pathlib.Path(args['overrideFile']).read_text()=='pVehicle.pickup[1,2]=3.0\\n'
assert 'steeringAngleCommand' in pathlib.Path(args['csvInput']).read_text()
pathlib.Path(args['r']).write_text('"time","velX"\\n0,1\\n0.2,2\\n')
print('pVehicle.pickup[1,2] = 3')
print('The simulation finished successfully')
''')
    executable.chmod(0o755)
    (tmp_path/'Vehicle_init.xml').write_text('''<fmiModelDescription>
<DefaultExperiment stopTime="1"/>
<ModelVariables>
<ScalarVariable name="pVehicle.pickup[1,2]" variability="parameter" causality="parameter" isValueChangeable="true"/>
<ScalarVariable name="velX" variability="continuous"/>
</ModelVariables></fmiModelDescription>''')
    inputs=np.array([(0.,0.),(.2,.1)],dtype=[('time',float),('steeringAngleCommand',float)])
    result,elapsed=verify.native_trajectory(executable,tmp_path/'run',
        {'pVehicle.pickup[1,2]':3.},inputs,['velX','pVehicle.pickup[1,2]'])
    np.testing.assert_array_equal(result['velX'],[1,2])
    np.testing.assert_array_equal(result['pVehicle.pickup[1,2]'],[3,3])
    assert elapsed>0
    assert 'stopTime="1"' in (tmp_path/'Vehicle_init.xml').read_text()
    with pytest.raises(ValueError,match='non-editable'):
        verify.native_trajectory(executable,tmp_path/'rejected',{'velX':3.},inputs,['velX'])


@pytest.mark.parametrize('causality', ['calculatedParameter', 'local', 'output'])
def test_native_rejects_non_parameter_even_if_marked_changeable(tmp_path, causality):
    executable = tmp_path / 'Vehicle'
    (tmp_path / 'Vehicle_init.xml').write_text(f'''<fmiModelDescription>
<DefaultExperiment stopTime="1"/>
<ModelVariables>
<ScalarVariable name="pVehicle.derived" causality="{causality}" isValueChangeable="true"/>
</ModelVariables></fmiModelDescription>''')
    inputs = np.array([(0., 0.), (.1, 0.)], dtype=[('time', float), ('steeringAngleCommand', float)])
    with pytest.raises(ValueError, match='non-editable'):
        verify.native_trajectory(executable, tmp_path / 'run', {'pVehicle.derived': 1.}, inputs, [])
