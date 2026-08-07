from __future__ import annotations

import csv
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
    "BobLib.Experiments.Standards.VehicleFMI",
    "BobLib.Experiments.Standards.Templates.FMI.BaseVehicleFMI",
    "BobLib.Experiments.Standards.VehicleSim",
    "BobLib.Experiments.Standards.FourPostSim",
    "BobLib.Experiments.Standards.Templates.FourPost.FourPostSim_DWDirect_DWDirect",
    "BobLib.Engines.SimpleICEngine",
    "BobLib.Transmissions.FixedRatioTransmission",
    "BobLib.Atmospheres.ConstantAtmosphere",
    "BobLib.Chassis.Brakes.BasicVCUBrakes",
    "BobLib.DriverEnvironments.Internal.Driver",
    "BobLib.DriverEnvironments.AutomaticDriveByWire",
    "BobLib.DriverEnvironments.EVDriveByWire",
    "BobLibTest.TestVehicle.TestPowertrain.TestBatteryInverter",
    "BobLibTest.TestVehicle.TestPowertrain.TestBatteryInverterMotor",
    "BobLibTest.TestVehicle.TestPowertrain.TestPowerLimitedMotor",
    "BobLibTest.TestVehicle.TestAero.TestCFDAeroMap",
    "BobLibTest.TestVehicle.TestPowertrain.TestVCU",
    "BobLibTest.TestVehicle.TestPowertrain.TestStandardVCU",
    "BobLibTest.TestVehicle.TestPowertrain.TestPowertrain",
    "BobLibTest.TestVehicle.TestChassis.TestSuspension.TestFrAxleDW",
    "BobLibTest.TestVehicle.TestChassis.TestSuspension.TestRrAxleDW",
    "BobLibTest.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestTire.TestFourMF52Kinematic",
    "BobLibTest.Regression.MF52PureSlipSmoke",
)

EXPECTED_EQUATION_VARIABLE_DIFFS = {
    "BobLib.Engines.SimpleICEngine": -1,
    "BobLib.Chassis.Brakes.BasicVCUBrakes": -1,
}

ROD_TO_LOWER_AXLE_FILES = (
    Path("BobLib/Chassis/Suspension/FrAxleDW_BC.mo"),
    Path("BobLib/Chassis/Suspension/FrAxleDW_BC_Stabar.mo"),
    Path("BobLib/Chassis/Suspension/FrAxleDW_Direct.mo"),
    Path("BobLib/Chassis/Suspension/RrAxleDW_BC.mo"),
    Path("BobLib/Chassis/Suspension/RrAxleDW_BC_Stabar.mo"),
    Path("BobLib/Chassis/Suspension/RrAxleDW_Direct.mo"),
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _render_mos(repo_root: Path, model: str) -> str:
    library_root = repo_root / "BobLib"
    tests_root = repo_root / "Tests" / "BobLibTest"
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
def test_boblib_model_translates(model: str) -> None:
    equation_count, variable_count = _check_model(model)
    print(f"{model}: {equation_count} equations, {variable_count} variables")


def test_nonzero_right_wheel_angles_are_mirrored() -> None:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    repo_root = _repo_root()
    library_root = repo_root / "BobLib"
    tests_root = repo_root / "Tests" / "BobLibTest"
    with tempfile.TemporaryDirectory() as temporary_directory:
        work_dir = Path(temporary_directory)
        probe_path = work_dir / "RightWheelMirroringProbe.mo"
        probe_path.write_text(
            """
model RightWheelChassisProbe

  extends BobLibTest.TestVehicle.TestChassis.TestSuspension.TestFrAxleDW(
    pVehicle(
      pFrPartialWheel(staticAlpha = 1.25, staticGamma = 2.5)));

  output Real frontLeftAlpha =
    frAxleDW.leftTire.pPartialWheel.staticAlpha;
  output Real frontRightAlpha =
    frAxleDW.rightTire.pPartialWheel.staticAlpha;
  output Real frontRightWheelAlpha =
    frAxleDW.rightTire.wheelModel.partialWheelParams.staticAlpha;
  output Real frontLeftGamma =
    frAxleDW.leftTire.pPartialWheel.staticGamma;
  output Real frontRightGamma =
    frAxleDW.rightTire.pPartialWheel.staticGamma;
  output Real frontRightWheelGamma =
    frAxleDW.rightTire.wheelModel.partialWheelParams.staticGamma;
  output Real frontHubXMirrorError[3] =
    Modelica.Mechanics.MultiBody.Frames.resolve1(
      frAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {1, 0, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        frAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {1, 0, 0}));
  output Real frontOutwardNormalMirrorError[3] =
    -Modelica.Mechanics.MultiBody.Frames.resolve1(
      frAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {0, 1, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        frAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {0, 1, 0}));
  output Real frontContactPatchMirrorError[3] =
    frAxleDW.rightCP.r_0 -
    BobLib.Utilities.Math.Vector.mirrorXZ(frAxleDW.leftCP.r_0);

end RightWheelChassisProbe;

model RightWheelFourPostProbe

  extends BobLib.Experiments.Standards.Templates.FourPost.FourPostSim_DWDirect_DWDirect(
    pVehicle(
      pFrPartialWheel(staticAlpha = 1.25, staticGamma = 2.5),
      pRrPartialWheel(staticAlpha = -0.75, staticGamma = 1.5)));

  output Real frontLeftAlpha =
    frAxleDW.leftTire.pPartialWheel.staticAlpha;
  output Real frontRightAlpha =
    frAxleDW.rightTire.pPartialWheel.staticAlpha;
  output Real frontRightWheelAlpha =
    frAxleDW.rightTire.wheelModel.partialWheelParams.staticAlpha;
  output Real frontLeftGamma =
    frAxleDW.leftTire.pPartialWheel.staticGamma;
  output Real frontRightGamma =
    frAxleDW.rightTire.pPartialWheel.staticGamma;
  output Real frontRightWheelGamma =
    frAxleDW.rightTire.wheelModel.partialWheelParams.staticGamma;

  output Real rearLeftAlpha =
    rrAxleDW.leftTire.pPartialWheel.staticAlpha;
  output Real rearRightAlpha =
    rrAxleDW.rightTire.pPartialWheel.staticAlpha;
  output Real rearRightWheelAlpha =
    rrAxleDW.rightTire.wheelModel.partialWheelParams.staticAlpha;
  output Real rearLeftGamma =
    rrAxleDW.leftTire.pPartialWheel.staticGamma;
  output Real rearRightGamma =
    rrAxleDW.rightTire.pPartialWheel.staticGamma;
  output Real rearRightWheelGamma =
    rrAxleDW.rightTire.wheelModel.partialWheelParams.staticGamma;
  output Real frontHubXMirrorError[3] =
    Modelica.Mechanics.MultiBody.Frames.resolve1(
      frAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {1, 0, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        frAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {1, 0, 0}));
  output Real frontOutwardNormalMirrorError[3] =
    -Modelica.Mechanics.MultiBody.Frames.resolve1(
      frAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {0, 1, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        frAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {0, 1, 0}));
  output Real frontContactPatchMirrorError[3] =
    frAxleDW.rightCP.r_0 -
    BobLib.Utilities.Math.Vector.mirrorXZ(frAxleDW.leftCP.r_0);
  output Real rearHubXMirrorError[3] =
    Modelica.Mechanics.MultiBody.Frames.resolve1(
      rrAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {1, 0, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        rrAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {1, 0, 0}));
  output Real rearOutwardNormalMirrorError[3] =
    -Modelica.Mechanics.MultiBody.Frames.resolve1(
      rrAxleDW.rightTire.wheelModel.hubAxis.frame_b.R,
      {0, 1, 0}) -
    BobLib.Utilities.Math.Vector.mirrorXZ(
      Modelica.Mechanics.MultiBody.Frames.resolve1(
        rrAxleDW.leftTire.wheelModel.hubAxis.frame_b.R,
        {0, 1, 0}));
  output Real rearContactPatchMirrorError[3] =
    rrAxleDW.rightCP.r_0 -
    BobLib.Utilities.Math.Vector.mirrorXZ(rrAxleDW.leftCP.r_0);

end RightWheelFourPostProbe;
""".strip()
            + "\n"
        )
        mos_path = work_dir / "right_wheel_mirroring_probe.mos"
        mos_path.write_text(
            f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});
loadFile("{library_root.as_posix()}/package.mo");
loadFile("{tests_root.as_posix()}/package.mo");
loadFile("{probe_path.as_posix()}");
cd("{work_dir.as_posix()}");
simulate(
  RightWheelChassisProbe,
  startTime=0,
  stopTime=0,
  numberOfIntervals=1,
  outputFormat="csv",
  variableFilter="time|front.*");
simulate(
  RightWheelFourPostProbe,
  startTime=0,
  stopTime=0,
  numberOfIntervals=1,
  outputFormat="csv",
  variableFilter="time|front.*|rear.*");
getErrorString();
""".strip()
            + "\n"
        )

        completed = subprocess.run(
            [omc, str(mos_path)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        assert completed.returncode == 0, completed.stdout

        results = {}
        for model in ("RightWheelChassisProbe", "RightWheelFourPostProbe"):
            result_path = work_dir / f"{model}_res.csv"
            assert result_path.is_file(), completed.stdout
            with result_path.open(newline="") as stream:
                results[model] = {
                    signal: float(value)
                    for signal, value in list(csv.DictReader(stream))[-1].items()
                    if signal != "time"
                }

    geometry_errors = {
        model: {
            signal: value
            for signal, value in signals.items()
            if "MirrorError[" in signal
        }
        for model, signals in results.items()
    }
    for model, errors in geometry_errors.items():
        assert errors, f"{model} did not publish geometry errors"
        assert errors == pytest.approx(
            {signal: 0.0 for signal in errors},
            abs=1e-10,
        )

    parameter_results = {
        model: {
            signal: value
            for signal, value in signals.items()
            if "MirrorError[" not in signal
        }
        for model, signals in results.items()
    }

    assert parameter_results["RightWheelChassisProbe"] == pytest.approx(
        {
            "frontLeftAlpha": 1.25,
            "frontRightAlpha": -1.25,
            "frontRightWheelAlpha": -1.25,
            "frontLeftGamma": 2.5,
            "frontRightGamma": -2.5,
            "frontRightWheelGamma": -2.5,
        }
    )
    assert parameter_results["RightWheelFourPostProbe"] == pytest.approx(
        {
            "frontLeftAlpha": 1.25,
            "frontRightAlpha": -1.25,
            "frontRightWheelAlpha": -1.25,
            "frontLeftGamma": 2.5,
            "frontRightGamma": -2.5,
            "frontRightWheelGamma": -2.5,
            "rearLeftAlpha": -0.75,
            "rearRightAlpha": 0.75,
            "rearRightWheelAlpha": 0.75,
            "rearLeftGamma": 1.5,
            "rearRightGamma": -1.5,
            "rearRightWheelGamma": -1.5,
        }
    )


@pytest.mark.parametrize(
    "relative_path",
    (
        Path("BobLib/Chassis/Chassis_DW.mo"),
        Path("BobLib/Chassis/Chassis_DWBCStabar_DWBCStabar.mo"),
        Path(
            "BobLib/Experiments/Standards/Templates/FourPost/"
            "BaseFourPostSim.mo"
        ),
    ),
    ids=str,
)
def test_right_tire_redeclarations_preserve_mirrored_record(
    relative_path: Path,
) -> None:
    source = (_repo_root() / relative_path).read_text()
    for axle in ("Fr", "Rr"):
        left_record = f"pVehicle.p{axle}PartialWheel"
        assert source.count(f"pPartialWheel = {left_record}") == 1
        assert source.count(f"partialWheelParams = {left_record}") == 1

    axle_base = (
        _repo_root() / "BobLib" / "Chassis" / "Suspension" / "AxleDWBase.mo"
    ).read_text()
    assert "staticAlpha = -pLeftPartialWheel.staticAlpha" in axle_base
    assert "staticGamma = -pLeftPartialWheel.staticGamma" in axle_base
    assert "rightTire(pPartialWheel = pRightPartialWheel)" in axle_base

    tire_base = (
        _repo_root()
        / "BobLib"
        / "Chassis"
        / "Suspension"
        / "Tires"
        / "BaseTire.mo"
    ).read_text()
    assert "wheelModel(partialWheelParams = pPartialWheel)" in tire_base


def test_base_vehicle_sim_replaceable_placements_are_canonical() -> None:
    source_path = (
        _repo_root()
        / "BobLib"
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
        / "BobLib"
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
