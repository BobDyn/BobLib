"""A steady-state sequence advances only after accepted holds."""
import re
import subprocess

import pytest
from test_vehicle_termination import vehicle_executable as build_vehicle


@pytest.fixture(scope="module")
def sequence_executable(tmp_path_factory):
    return build_vehicle.__wrapped__(tmp_path_factory)


def test_sequence_preserves_time_and_reports_every_acceptance(sequence_executable):
    work = sequence_executable
    overrides = ('enableSteadyStateSequence=true,steadyStatePointCount=3,'
                 'driver.useMode=3,initialVel=15,driver.steerStart=0.01,'
                 'driver.linearitySlopeSamplePeriod=0.02,steadyStateHoldDuration=0.1,'
                 'steadyStateSpeedTolerance=1')
    result = subprocess.run([str(work / 'vehicle'), '-override=' + overrides,
                             '-stopTime=10', '-r=sequence.csv', '-noEventEmit'],
                            cwd=work, text=True, capture_output=True, timeout=180)
    output = result.stdout + result.stderr
    (work / 'sequence.log').write_text(output)
    assert result.returncode == 0, output
    assert 'override variable name not found' not in output, output
    points = re.findall(r'BOBLIB_STEADY_POINT: index=(\d+) time=([\d.eE+-]+) target=([\d.eE+-]+)', output)
    assert [int(i) for i, _, _ in points] == [1, 2, 3], output
    times = [float(t) for _, t, _ in points]
    assert all(b - a >= 0.1 for a, b in zip(times, times[1:], strict=False)), output
    assert 'Reached closed-loop steady-state sequence targets' in output


def test_sequence_rejects_nonsteady_maneuver(sequence_executable):
    work = sequence_executable
    result = subprocess.run([str(work / 'vehicle'),
                             '-override=enableSteadyStateSequence=true,driver.useMode=2',
                             '-stopTime=0.1', '-r=invalid_sequence.csv'],
                            cwd=work, text=True, capture_output=True, timeout=90)
    assert result.returncode != 0
    assert 'BOBLIB_INVALID_SEQUENCE' in result.stdout + result.stderr


def test_sequence_reaches_nonzero_targets(sequence_executable):
    import csv
    work = sequence_executable
    overrides = ('enableSteadyStateSequence=true,steadyStatePointCount=3,'
                 'steadyStateTargets[1]=0,steadyStateTargets[2]=0.2,steadyStateTargets[3]=0.4,'
                 'driver.useMode=3,initialVel=15,driver.steerStart=0.01,'
                 'steadyStateHoldDuration=0.3,driver.steadyStateAyGain=0.05,'
                 'driver.steadyStateAyTi=1,driver.velGain=100,driver.velTi=2')
    result = subprocess.run([str(work / 'vehicle'), '-override=' + overrides,
                             '-stopTime=30', '-r=nonzero_sequence.csv',
                             '-variableFilter=time|accY|yawVel|handwheelAngle|velX|steadyStateSpeedRate'],
                            cwd=work, text=True, capture_output=True, timeout=300)
    output = result.stdout + result.stderr
    (work / 'nonzero_sequence.log').write_text(output)
    assert result.returncode == 0, output
    points = re.findall(r'BOBLIB_STEADY_POINT: index=(\d+) time=([\d.eE+-]+) target=([\d.eE+-]+)', output)
    assert [float(target) for _, _, target in points] == [0, 0.2, 0.4], output
    assert 'Reached closed-loop steady-state sequence targets' in output
    with (work / 'nonzero_sequence.csv').open() as stream:
        final = list(csv.DictReader(stream))[-1]
    assert abs(float(final['accY']) - 0.4) <= 0.051, final
    assert abs(float(final['steadyStateSpeedRate'])) <= 0.02001, final
