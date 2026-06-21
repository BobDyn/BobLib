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
    "-d=initialization,NLSanalyticJacobian,disableStartCalc "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)

CHECK_MODELS = (
    "BobLibVehicleInterfaces.Experiments.Standards.VehicleFMI",
    "BobLibVehicleInterfaces.Experiments.Standards.Templates.FMI.BaseVehicleFMI",
    "BobLibVehicleInterfaces.Experiments.Standards.VehicleSim",
    "BobLibVehicleInterfaces.Experiments.Standards.FourPostSim",
    "BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPost.FourPostSim_DWDirect_DWDirect",
    "BobLibVehicleInterfaces.Engines.SimpleICEngine",
    "BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission",
    "BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere",
    "BobLibVehicleInterfaces.Chassis.Brakes.BasicVCUBrakes",
    "BobLibVehicleInterfaces.DriverEnvironments.Internal.Driver",
    "BobLibVehicleInterfaces.DriverEnvironments.AutomaticDriveByWire",
    "BobLibVehicleInterfaces.DriverEnvironments.EVDriveByWire",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestBatteryInverter",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestBatteryInverterMotor",
    "BobLibVehicleInterfacesTests.TestVehicle.TestAero.TestCFDAeroMap",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestVCU",
    "BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain.TestPowertrain",
    "BobLibVehicleInterfacesTests.TestVehicle.TestChassis.TestSuspension.TestFrAxleDW",
    "BobLibVehicleInterfacesTests.TestVehicle.TestChassis.TestSuspension.TestRrAxleDW",
    "BobLibVehicleInterfacesTests.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestTire.TestFourMF52Kinematic",
    "BobLibVehicleInterfacesTests.Regression.MF52PureSlipSmoke",
)

EXPECTED_EQUATION_VARIABLE_DIFFS = {
    "BobLibVehicleInterfaces.Engines.SimpleICEngine": -1,
    "BobLibVehicleInterfaces.Chassis.Brakes.BasicVCUBrakes": -1,
    "BobLibVehicleInterfacesTests.TestVehicle.TestAero.TestCFDAeroMap": -3,
}

ROD_TO_LOWER_AXLE_FILES = (
    Path("BobLibVehicleInterfaces/Chassis/Suspension/FrAxleDW_BC.mo"),
    Path("BobLibVehicleInterfaces/Chassis/Suspension/FrAxleDW_BC_Stabar.mo"),
    Path("BobLibVehicleInterfaces/Chassis/Suspension/FrAxleDW_Direct.mo"),
    Path("BobLibVehicleInterfaces/Chassis/Suspension/RrAxleDW_BC.mo"),
    Path("BobLibVehicleInterfaces/Chassis/Suspension/RrAxleDW_BC_Stabar.mo"),
    Path("BobLibVehicleInterfaces/Chassis/Suspension/RrAxleDW_Direct.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/FrAxleDW_BC.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/FrAxleDW_BC_Stabar.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/FrAxleDW_Direct.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/RrAxleDW_BC.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/RrAxleDW_BC_Stabar.mo"),
    Path("BobLib/Vehicle/Chassis/Suspension/RrAxleDW_Direct.mo"),
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


def _check_model(model: str) -> tuple[int, int]:
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
        rf"Class {re.escape(model)} has ([0-9]+) equation\(s\) and ([0-9]+) variable\(s\)",
        output,
    )
    assert match is not None, f"Could not parse equation/variable count for {model}:\n{output}"
    equation_count = int(match.group(1))
    variable_count = int(match.group(2))
    expected_diff = EXPECTED_EQUATION_VARIABLE_DIFFS.get(model, 0)
    actual_diff = equation_count - variable_count
    assert actual_diff == expected_diff, (
        f"{model} has unexpected equation/variable balance: "
        f"{equation_count} equations vs {variable_count} variables "
        f"(diff {actual_diff}, expected {expected_diff})\n{output}"
    )
    return equation_count, variable_count


@pytest.mark.parametrize("model", CHECK_MODELS, ids=CHECK_MODELS)
def test_boblibvehicleinterfaces_model_translates(model: str) -> None:
    equation_count, variable_count = _check_model(model)
    print(f"{model}: {equation_count} equations, {variable_count} variables")


def test_base_vehicle_sim_replaceable_placements_are_canonical() -> None:
    source_path = (
        _repo_root()
        / "BobLibVehicleInterfaces"
        / "Experiments"
        / "Standards"
        / "Templates"
        / "Vehicle"
        / "BaseVehicleSim.mo"
    )
    source = source_path.read_text()
    bad_fragments = []
    for match in re.finditer(r"\bconstrainedby\b", source):
        declaration_start = source.rfind(";", 0, match.start()) + 1
        declaration_end = source.find(";", match.start())
        declaration = source[declaration_start:declaration_end]
        pre_constrainedby = declaration[: match.start() - declaration_start]
        post_constrainedby = declaration[match.end() - declaration_start :]
        pre_annotation_count = pre_constrainedby.count("annotation")
        post_annotation_count = post_constrainedby.count("annotation")
        if pre_annotation_count != 1 or post_annotation_count != 0:
            line = source.count("\n", 0, declaration_start) + 1
            bad_fragments.append((line, declaration.strip()))

    assert not bad_fragments, (
        "BaseVehicleSim replaceable constrained components must keep their "
        "single Placement annotation before constrainedby. OMEdit edits that "
        "pre-constrainedby slot; a second post-constrainedby annotation can "
        "make moved icons or resized icons reload from stale geometry.\n"
        + "\n".join(f"line {line}: {fragment}" for line, fragment in bad_fragments)
    )


def test_base_vehicle_sim_origin_placements_use_local_extents() -> None:
    source_path = (
        _repo_root()
        / "BobLibVehicleInterfaces"
        / "Experiments"
        / "Standards"
        / "Templates"
        / "Vehicle"
        / "BaseVehicleSim.mo"
    )
    source = source_path.read_text()
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    placement_pattern = re.compile(
        rf"Placement\s*\(\s*transformation\s*\(\s*origin\s*=\s*\{{\s*"
        rf"(?P<ox>{number})\s*,\s*(?P<oy>{number})\s*\}}\s*,\s*"
        rf"extent\s*=\s*\{{\{{\s*(?P<x1>{number})\s*,\s*(?P<y1>{number})\s*\}}\s*,\s*"
        rf"\{{\s*(?P<x2>{number})\s*,\s*(?P<y2>{number})\s*\}}\s*\}}",
        re.DOTALL,
    )
    bad_fragments = []
    for match in placement_pattern.finditer(source):
        x1 = float(match.group("x1"))
        y1 = float(match.group("y1"))
        x2 = float(match.group("x2"))
        y2 = float(match.group("y2"))
        if abs(x1 + x2) > 1e-4 or abs(y1 + y2) > 1e-4:
            line = source.count("\n", 0, match.start()) + 1
            bad_fragments.append((line, match.group(0).strip()))

    assert not bad_fragments, (
        "BaseVehicleSim placements with an explicit origin must use local "
        "extents centered on that origin. Offset extents can reload with "
        "size discrepancies in OMEdit.\n"
        + "\n".join(f"line {line}: {fragment}" for line, fragment in bad_fragments)
    )


@pytest.mark.parametrize("relative_path", ROD_TO_LOWER_AXLE_FILES, ids=str)
def test_axles_preserve_rod_to_lower_attachment_selector(relative_path: Path) -> None:
    source_path = _repo_root() / relative_path
    source = source_path.read_text()

    required_snippets = (
        "leftRodWishboneMount[3] = if pAxle.rodToLower then pLeftDW.lower_o else pLeftDW.upper_o",
        "r = pAxle.rodMount - leftRodWishboneMount",
        "r = mirrorXZ(pAxle.rodMount - leftRodWishboneMount)",
        "if pAxle.rodToLower then",
        "connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o)",
        "connect(toRightApex.frame_a, rightWishboneUprightLoop.lowerFrame_o)",
        "connect(toLeftApex.frame_a, leftWishboneUprightLoop.upperFrame_o)",
        "connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o)",
    )

    missing = [snippet for snippet in required_snippets if snippet not in source]
    assert not missing, (
        f"{relative_path} must keep pAxle.rodToLower wired through both the "
        "apex translation and upper/lower wishbone frame connections. Missing:\n"
        + "\n".join(missing)
    )
