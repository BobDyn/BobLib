"""The physical voltage-cutoff regression must also survive an FMI export."""
import os
import shutil
from pathlib import Path

import pytest

import modelica_fmu_export


@pytest.mark.skipif(os.environ.get('BOBLIB_TEST_FMU_RUNTIME') != '1',
                    reason='Set BOBLIB_TEST_FMU_RUNTIME=1 to export the voltage-limit regression')
def test_voltage_limits_are_well_posed_in_cosimulation(tmp_path):
    if any(shutil.which(tool) is None for tool in ('omc', 'clang', 'cmake')):
        pytest.skip('OpenModelica, Clang and CMake are required')
    fmpy = pytest.importorskip('fmpy')
    np = pytest.importorskip('numpy')
    root = Path(__file__).resolve().parents[1]
    fixture = root / 'Tests/BobLibTest/PowerElectronics/Internal/TestInverterDCCore.mo'
    source = fixture.read_text().split('\n', 1)[1].replace('TestInverterDCCore', 'VoltageProtection')
    artifact = modelica_fmu_export.export_fmu(
        package_root=root / 'BobLib', output_dir=tmp_path, timeout_s=300,
        name='VoltageProtection', model=source)
    result = fmpy.simulate_fmu(str(artifact), fmi_type='CoSimulation', stop_time=.05,
                               output_interval=.001, relative_tolerance=1e-7,
                               output=['fullBusVoltage', 'lowBusVoltage', 'regenPower', 'drivePower'])
    assert result['time'][-1] == pytest.approx(.05)
    assert all(np.isfinite(result[name]).all() for name in result.dtype.names)
    assert np.max(result['fullBusVoltage']) <= 588 + 1e-5
    assert np.min(result['lowBusVoltage']) >= 200 - 1e-5
    assert -100 < result['regenPower'][-1] < 0
    assert 0 < result['drivePower'][-1] < 10000
