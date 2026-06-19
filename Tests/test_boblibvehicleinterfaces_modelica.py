from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest


MODELICA_VERSION = "4.1.0"
VEHICLE_INTERFACES_VERSION = "2.0.2"
OMC_COMMAND_LINE_OPTIONS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "-d=initialization,NLSanalyticJacobian "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)

CHECK_MODELS = (
    "BobLibVehicleInterfaces.Experiments.Standards.VehicleSim",
    "BobLibVehicleInterfaces.Experiments.Standards.FourPostSim",
    "BobLibVehicleInterfaces.Engines.SimpleICEngine",
    "BobLibVehicleInterfaces.Engines.ICEnginePlaceholder",
    "BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission",
    "BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere",
    "BobLibVehicleInterfaces.Chassis.Brakes.BasicVCUBrakes",
    "BobLibVehicleInterfaces.DriverEnvironments.AutomaticDriveByWire",
    "BobLibVehicleInterfaces.DriverEnvironments.EVDriveByWire",
    "BobLibVehicleInterfacesTests.TestVehicle.TestAero.TestCFDAeroMap",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestVCU",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestPowertrain",
    "BobLibVehicleInterfacesTests.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestTire.TestFourMF52Kinematic",
    "BobLibVehicleInterfacesTests.Regression.MF52PureSlipSmoke",
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _render_mos(repo_root: Path, model: str) -> str:
    library_root = repo_root / "BobLibVehicleInterfaces"
    tests_root = repo_root / "BobLibVehicleInterfacesTests"
    lines = [
        "clear();",
        f'setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");',
        f'loadModel(Modelica, {{"{MODELICA_VERSION}"}});',
        f'loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});',
        f'loadFile("{library_root.as_posix()}/package.mo");',
        f'loadFile("{tests_root.as_posix()}/package.mo");',
    ]
    lines.extend((f"checkModel({model});", "getErrorString();"))
    return "\n".join(lines) + "\n"


def _check_model(model: str) -> int:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    with tempfile.NamedTemporaryFile("w", suffix=".mos", delete=False) as mos:
        mos.write(_render_mos(_repo_root(), model))
        mos_path = Path(mos.name)

    try:
        completed = subprocess.run(
            [omc, str(mos_path)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        mos_path.unlink(missing_ok=True)

    output = completed.stdout
    if completed.returncode != 0:
        pytest.fail(f"omc failed for {model}:\n{output}")

    success = f"Check of {model} completed successfully."
    assert success in output, f"checkModel did not report success for {model}:\n{output}"
    match = re.search(
        rf"Class {re.escape(model)} has ([0-9]+) equation\(s\)",
        output,
    )
    assert match is not None, f"Could not parse equation count for {model}:\n{output}"
    return int(match.group(1))


@pytest.mark.parametrize("model", CHECK_MODELS, ids=CHECK_MODELS)
def test_boblibvehicleinterfaces_model_translates(model: str) -> None:
    equation_count = _check_model(model)
    print(f"{model}: {equation_count} equations")
