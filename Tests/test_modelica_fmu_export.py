from __future__ import annotations

import math
import os
import shutil
from pathlib import Path

import pytest

import modelica_fmu_export as fmu_export

fmpy = pytest.importorskip("fmpy")
np = pytest.importorskip("numpy")

DRIVER_INPUTS = ("steeringAngleCommand", "acceleratorPedalCommand", "brakePedalCommand")

# x rises at 1/s and resets at 1, so 8.5 s hold exactly 8 resets. A state event
# found only at the end of each 0.3 s communication step makes every reset
# late, and one of them falls past the end.
RESET_PROBE = """model ResetProbe
  Real x(start = 0, fixed = true);
  Integer resets(start = 0, fixed = true);
equation
  der(x) = 1;
  when x >= 1 then
    reinit(x, 0);
    resets = pre(resets) + 1;
  end when;
end ResetProbe;"""

# x = exp(-t), so the error at the end shows the tolerance CVODE used.
DECAY_PROBE = """model DecayProbe
  Real x(start = 1, fixed = true);
equation
  der(x) = -x;
end DecayProbe;"""


def _configured_package_root(config: pytest.Config) -> Path:
    configured = config.getoption("--boblib-package-root")
    if configured:
        package_root = Path(configured)
        if not package_root.is_absolute():
            package_root = Path.cwd() / package_root
        return package_root.resolve()
    return fmu_export.default_package_root().resolve()


def _export(
    config: pytest.Config,
    tmp_path_factory: pytest.TempPathFactory,
    name: str = fmu_export.FMU_NAME,
    model: str = fmu_export.EXPORT_MODEL,
) -> Path:
    if os.name == "nt":
        pytest.skip("The FMU export does not support Windows yet")
    if shutil.which("omc") is None:
        pytest.skip("OpenModelica omc is not installed")
    if shutil.which("cmake") is None:
        pytest.skip("CMake is not installed")

    return fmu_export.export_fmu(
        _configured_package_root(config),
        tmp_path_factory.mktemp("fmu"),
        timeout_s=config.getoption("--boblib-fmu-timeout-s") or fmu_export.DEFAULT_TIMEOUT_S,
        name=name,
        model=model,
    )


@pytest.fixture(scope="module")
def vehicle_fmu(pytestconfig: pytest.Config, tmp_path_factory: pytest.TempPathFactory) -> Path:
    return _export(pytestconfig, tmp_path_factory)


@pytest.fixture(scope="module")
def reset_probe_fmu(
    pytestconfig: pytest.Config,
    tmp_path_factory: pytest.TempPathFactory,
) -> Path:
    return _export(pytestconfig, tmp_path_factory, name="ResetProbe", model=RESET_PROBE)


@pytest.fixture(scope="module")
def decay_probe_fmu(
    pytestconfig: pytest.Config,
    tmp_path_factory: pytest.TempPathFactory,
) -> Path:
    return _export(pytestconfig, tmp_path_factory, name="DecayProbe", model=DECAY_PROBE)


def test_state_events_are_found_inside_a_step(reset_probe_fmu: Path) -> None:
    result = fmpy.simulate_fmu(
        str(reset_probe_fmu),
        fmi_type="CoSimulation",
        stop_time=8.5,
        output_interval=0.3,
        output=["x", "resets"],
    )

    assert result["resets"][-1] == 8
    assert math.isclose(result["x"][-1], 0.5, abs_tol=1e-4), result["x"][-1]


def test_chattering_guard_defers_events_to_the_end_of_a_step(reset_probe_fmu: Path) -> None:
    # 125 resets fall in each 125 s step. After CHATTERING_STATE_EVENTS of them
    # the step stops searching and handles the next one at its end, so each step
    # adds one more than the limit; the search resumes in the next step.
    result = fmpy.simulate_fmu(
        str(reset_probe_fmu),
        fmi_type="CoSimulation",
        stop_time=250.0,
        output_interval=125.0,
        output=["resets"],
    )

    per_step = fmu_export.CHATTERING_STATE_EVENTS + 1
    assert list(result["resets"]) == [0, per_step, 2 * per_step]


def test_requested_tolerance_reaches_cvode(decay_probe_fmu: Path) -> None:
    errors = {}
    for tolerance in (1e-2, 1e-9):
        result = fmpy.simulate_fmu(
            str(decay_probe_fmu),
            fmi_type="CoSimulation",
            stop_time=5.0,
            output_interval=1.0,
            relative_tolerance=tolerance,
            output=["x"],
        )
        errors[tolerance] = abs(result["x"][-1] - math.exp(-5.0))

    # Before the tolerance edit both runs used the default 1e-6 and erred by 7.8e-7.
    assert errors[1e-9] < 1e-7, errors
    assert errors[1e-2] > 100 * errors[1e-9], errors


def test_vehicle_fmu_exposes_driver_inputs(vehicle_fmu: Path) -> None:
    description = fmpy.read_model_description(str(vehicle_fmu))

    assert description.fmiVersion == "2.0"
    assert description.modelExchange is not None
    assert description.coSimulation is not None
    inputs = {
        variable.name
        for variable in description.modelVariables
        if variable.causality == "input"
    }
    assert inputs == set(DRIVER_INPUTS)


def test_vehicle_fmu_drives_from_standstill(vehicle_fmu: Path) -> None:
    # Hold the brake, drive off, steer left and back, then brake. Driving off
    # after holding the brake is what failed before the CVODE edits in
    # modelica_fmu_export. Inputs ramp, as a driver would move them.
    signals = np.array(
        [
            (0.0, 0.0, 0.0, 0.0),
            (0.5, 0.0, 0.0, 0.3),
            (3.0, 0.0, 0.0, 0.3),
            (3.5, 0.0, 0.3, 0.0),
            (4.5, 0.0, 0.3, 0.0),
            (5.0, 0.5, 0.3, 0.0),
            (5.5, 0.5, 0.3, 0.0),
            (6.0, 0.0, 0.3, 0.0),
            (6.5, 0.0, 0.0, 0.3),
            (7.0, 0.0, 0.0, 0.3),
        ],
        dtype=[("time", np.float64)] + [(name, np.float64) for name in DRIVER_INPUTS],
    )
    result = fmpy.simulate_fmu(
        str(vehicle_fmu),
        fmi_type="CoSimulation",
        stop_time=7.0,
        output_interval=0.01,
        input=signals,
        output=["vehicleSpeed", "yawVel"],
    )

    time = result["time"]
    speed = result["vehicleSpeed"]
    yaw_rate = result["yawVel"]
    assert math.isclose(time[-1], 7.0)
    assert np.isfinite(speed).all()
    assert np.isfinite(yaw_rate).all()

    held = float(np.abs(speed[time <= 3.0]).max())
    turning = float(yaw_rate[(time >= 5.0) & (time <= 6.0)].max())
    speed_at_brake = float(np.interp(6.0, time, speed))
    assert held < 0.05, held
    assert speed_at_brake > 3.0, speed_at_brake
    assert turning > 0.1, turning
    assert speed[-1] < speed_at_brake - 2.0, (speed_at_brake, speed[-1])


def test_vehicle_fmu_stops_and_goes_without_a_requested_tolerance(
    vehicle_fmu: Path,
    tmp_path: Path,
) -> None:
    # fmi2SetupExperiment without a tolerance, as many importers call it: drive
    # and brake in turn for about 3 s each, five times. Without the
    # zero-crossing hysteresis edit, the motor direction latch chatters and a
    # step fails, but only with time accumulated as below (measured on
    # aarch64); with time = i * step the drive passed without the edit too.
    from fmpy.fmi2 import FMU2Slave

    description = fmpy.read_model_description(str(vehicle_fmu))
    references = {variable.name: variable.valueReference for variable in description.modelVariables}
    inputs = [references[name] for name in DRIVER_INPUTS]
    fmu = FMU2Slave(
        guid=description.guid,
        unzipDirectory=fmpy.extract(str(vehicle_fmu), unzipdir=str(tmp_path)),
        modelIdentifier=description.coSimulation.modelIdentifier,
        instanceName="stop_and_go",
    )
    fmu.instantiate()
    try:
        fmu.setupExperiment(startTime=0.0)
        fmu.enterInitializationMode()
        fmu.exitInitializationMode()
        step = 0.02
        time = accelerator = brake = 0.0
        speeds = []
        for _ in range(1500):
            driving = int(time / 3.0) % 2 == 0
            # Pedals are pressed at most 2 per second and released at once.
            accelerator = min(0.3 if driving else 0.0, accelerator + 2.0 * step)
            brake = min(0.0 if driving else 0.4, brake + 2.0 * step)
            fmu.setReal(inputs, [0.2 * math.sin(time), accelerator, brake])
            fmu.doStep(currentCommunicationPoint=time, communicationStepSize=step)
            time += step
            speeds.append(fmu.getReal([references["vehicleSpeed"]])[0])
    finally:
        fmu.freeInstance()

    speed = np.array(speeds)
    assert np.isfinite(speed).all()
    # Moving at the end of each drive phase, nearly stopped at the end of each
    # braking phase.
    assert (speed[[149, 449, 749, 1049, 1349]] > 5.0).all(), speed[[149, 449, 749, 1049, 1349]]
    assert (speed[[299, 599, 899, 1199, 1499]] < 0.1).all(), speed[[299, 599, 899, 1199, 1499]]
