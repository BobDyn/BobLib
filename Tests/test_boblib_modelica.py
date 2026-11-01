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
    "BobLib.Experiments.Standards.BaseClasses.PartialVehicleFMI",
    "BobLib.Experiments.Standards.VehicleSim",
    "BobLib.Experiments.Standards.FourPostSim",
    "BobLibTest.Experiments.Standards.Configurations.FourPostSim_DWDirect_DWDirect",
    "BobLib.Engines.SimpleICEngine",
    "BobLib.Transmissions.FixedRatioTransmission",
    "BobLib.Atmospheres.ConstantAtmosphere",
    "BobLib.Brakes.BasicVCUBrakes",
    "BobLib.DriverEnvironments.Internal.Driver",
    "BobLib.DriverEnvironments.AutomaticDriveByWire",
    "BobLib.DriverEnvironments.EVDriveByWire",
    "BobLibTest.PowerElectronics.TestBatteryInverter",
    "BobLibTest.ElectricDrives.TestBatteryInverterMotor",
    "BobLibTest.ElectricDrives.Internal.TestPowerLimitedMotor",
    "BobLibTest.Aero.TestCFDAeroMap",
    "BobLibTest.Controllers.TestVCU",
    "BobLibTest.Drivers.TestStandardDriver",
    "BobLibTest.Drivelines.TestPowertrain",
    "BobLibTest.Chassis.Suspension.TestFrAxleDW",
    "BobLibTest.Chassis.Suspension.TestRrAxleDW",
    "BobLibTest.Chassis.Suspension.Tires.TestFourMF52Kinematic",
    "BobLibTest.Chassis.Suspension.Tires.MF52.MF52PureSlipSmoke",
)

EXPECTED_EQUATION_VARIABLE_DIFFS = {
    "BobLib.Engines.SimpleICEngine": -1,
    "BobLib.Brakes.BasicVCUBrakes": -1,
}

ROD_TO_LOWER_AXLE_FILES = (
    Path("BobLib/Chassis/Suspension/DoubleWishboneBellcrank.mo"),
    Path("BobLib/Chassis/Suspension/DoubleWishboneBellcrankStabar.mo"),
    Path("BobLib/Chassis/Suspension/DoubleWishboneDirect.mo"),
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


def test_vcu_enforces_motoring_power_limit() -> None:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    repo_root = _repo_root()
    library_root = repo_root / "BobLib"
    with tempfile.TemporaryDirectory() as temporary_directory:
        work_dir = Path(temporary_directory)
        probe_path = work_dir / "VCUPowerLimitProbe.mo"
        probe_path.write_text(
            """
model VCUPowerLimitProbe

  BobLib.Controllers.Internal.VCUCore vcu(pVCU(tau_max = 200,P_max_mot = 80000,w_eps = 0.1));
  Modelica.Blocks.Sources.Constant torqueRequest(k = 200);
  Modelica.Blocks.Sources.Constant zero(k = 0);
  Modelica.Blocks.Sources.Constant motorSpeed(k = 1000);
  Modelica.Blocks.Sources.BooleanConstant enabled(k = true);

equation
  connect(torqueRequest.y, vcu.cmd_torque_motor);
  connect(zero.y, vcu.cmd_regen_limit);
  connect(enabled.y, vcu.cmd_inverter_enable);
  connect(motorSpeed.y, vcu.sens_motor_speed);

  assert(abs(vcu.tau_cmd_limited - 80) < 1e-9,
    "VCU did not limit 200 Nm at 1000 rad/s to 80 Nm");
  assert(abs(vcu.P_req - 80000) < 1e-6,
    "VCU motoring request exceeded 80 kW");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end VCUPowerLimitProbe;
""".strip()
            + "\n",
            encoding="utf-8",
        )
        mos_path = work_dir / "vcu_power_limit_probe.mos"
        mos_path.write_text(
            f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});
loadFile("{library_root.as_posix()}/package.mo");
loadFile("{probe_path.as_posix()}");
cd("{work_dir.as_posix()}");
simulate(VCUPowerLimitProbe, startTime = 0, stopTime = 0.01, numberOfIntervals = 1);
getErrorString();
""".strip()
            + "\n",
            encoding="utf-8",
        )
        completed = subprocess.run(
            [omc, str(mos_path)],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

    assert completed.returncode == 0, completed.stdout
    assert 'messages = "LOG_SUCCESS' in completed.stdout, completed.stdout


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

  extends BobLibTest.Chassis.Suspension.TestFrAxleDW(
    pVehicle(      chassis(      frontAxle(      tire(      wheel(geometry(staticAlpha = 1.25, staticGamma = 2.5)))))));

  output Real frontLeftAlpha =
    frAxleDW.leftTire.pTire.wheel.geometry.staticAlpha;
  output Real frontRightAlpha =
    frAxleDW.rightTire.pTire.wheel.geometry.staticAlpha;
  output Real frontRightWheelAlpha =
    frAxleDW.rightTire.wheelModel.pWheel.geometry.staticAlpha;
  output Real frontLeftGamma =
    frAxleDW.leftTire.pTire.wheel.geometry.staticGamma;
  output Real frontRightGamma =
    frAxleDW.rightTire.pTire.wheel.geometry.staticGamma;
  output Real frontRightWheelGamma =
    frAxleDW.rightTire.wheelModel.pWheel.geometry.staticGamma;
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

  extends BobLibTest.Experiments.Standards.Configurations.FourPostSim_DWDirect_DWDirect(
    pVehicle(      chassis(      frontAxle(      tire(      wheel(geometry(staticAlpha = 1.25, staticGamma = 2.5)))),
      rearAxle(      tire(      wheel(geometry(staticAlpha = -0.75, staticGamma = 1.5)))))));

  output Real frontLeftAlpha =
    frAxleDW.leftTire.pTire.wheel.geometry.staticAlpha;
  output Real frontRightAlpha =
    frAxleDW.rightTire.pTire.wheel.geometry.staticAlpha;
  output Real frontRightWheelAlpha =
    frAxleDW.rightTire.wheelModel.pWheel.geometry.staticAlpha;
  output Real frontLeftGamma =
    frAxleDW.leftTire.pTire.wheel.geometry.staticGamma;
  output Real frontRightGamma =
    frAxleDW.rightTire.pTire.wheel.geometry.staticGamma;
  output Real frontRightWheelGamma =
    frAxleDW.rightTire.wheelModel.pWheel.geometry.staticGamma;

  output Real rearLeftAlpha =
    rrAxleDW.leftTire.pTire.wheel.geometry.staticAlpha;
  output Real rearRightAlpha =
    rrAxleDW.rightTire.pTire.wheel.geometry.staticAlpha;
  output Real rearRightWheelAlpha =
    rrAxleDW.rightTire.wheelModel.pWheel.geometry.staticAlpha;
  output Real rearLeftGamma =
    rrAxleDW.leftTire.pTire.wheel.geometry.staticGamma;
  output Real rearRightGamma =
    rrAxleDW.rightTire.pTire.wheel.geometry.staticGamma;
  output Real rearRightWheelGamma =
    rrAxleDW.rightTire.wheelModel.pWheel.geometry.staticGamma;
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
getErrorString();
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
            if re.fullmatch(r"(front|rear)(Left|Right|RightWheel)(Alpha|Gamma)", signal)
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
        Path("BobLib/Chassis/DoubleWishbone.mo"),
        Path(
            "BobLib/Experiments/Standards/BaseClasses/"
            "PartialDoubleWishboneFourPost.mo"
        ),
    ),
    ids=str,
)
def test_right_tire_redeclarations_preserve_mirrored_record(
    relative_path: Path,
) -> None:
    source = (_repo_root() / relative_path).read_text()
    # Axle owns mirroring; assemblies must not overwrite the right tire record.
    assert not re.search(r"rightTire\s*\(\s*pTire", source)

    axle_base = (
        _repo_root() / "BobLib" / "Chassis" / "Suspension" / "BaseClasses" / "PartialDoubleWishboneAxle.mo"
    ).read_text()
    assert "staticAlpha = -pAxle.tire.wheel.geometry.staticAlpha" in axle_base
    assert "staticGamma = -pAxle.tire.wheel.geometry.staticGamma" in axle_base
    assert re.search(r"\brightTire\b[^;]*?\bconstrainedby\s+[\w.]+BaseTire\(\s*pTire\s*=\s*rightTireCalibration\b", axle_base)

    tire_base = (
        _repo_root()
        / "BobLib"
        / "Chassis"
        / "Suspension"
        / "Tires"
        / "BaseTire.mo"
    ).read_text()
    assert re.search(r"wheelModel\s+constrainedby\s+[\w.]+PartialWheel\(\s*pWheel\s*=\s*pTire.wheel\b", tire_base)


def test_default_vehicle_mass_properties_include_both_axle_sides() -> None:
    omc = shutil.which("omc")
    if omc is None:
        pytest.skip("OpenModelica omc is not installed")

    repo_root = _repo_root()
    library_root = repo_root / "BobLib"
    with tempfile.TemporaryDirectory() as temporary_directory:
        work_dir = Path(temporary_directory)
        probe_path = work_dir / "ChassisMassPropertiesProbe.mo"
        result_path = work_dir / "ChassisMassPropertiesProbe_res.csv"
        probe_path.write_text(
            """
model ChassisMassPropertiesProbe

  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  // shortcut: avoid OMC 1.27.1 nested-constructor cycle; see compiler_probes/nested_record_defaults.mos.
  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle;
  final parameter MassRecord pVehicleMass(
    m(fixed=false), rCM(each fixed=false), inertia(each fixed=false));

  output Real totalMass = pVehicleMass.m;
  output Real vehicleCG[3] = pVehicleMass.rCM;

initial equation
  // Match the chassis initialization contract; whole-record bindings freeze in OMC.
  pVehicleMass =
    BobLib.Utilities.Mechanics.Functions.combineSymmetricAxleMassRecords(
      pVehicle.chassis.frame.mass,
      {pVehicle.chassis.frontAxle.masses, pVehicle.chassis.rearAxle.masses});

end ChassisMassPropertiesProbe;
""".strip()
            + "\n"
        )
        mos_path = work_dir / "chassis_mass_properties_probe.mos"
        mos_path.write_text(
            f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});
loadFile("{library_root.as_posix()}/package.mo");
loadFile("{probe_path.as_posix()}");
cd("{work_dir.as_posix()}");
simulate(
  ChassisMassPropertiesProbe,
  startTime=0,
  stopTime=0,
  numberOfIntervals=1,
  outputFormat="csv",
  variableFilter="time|totalMass|vehicleCG.*");
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
        assert result_path.is_file(), completed.stdout

        with result_path.open(newline="") as stream:
            final = list(csv.DictReader(stream))[-1]

    assert float(final["totalMass"]) == pytest.approx(261.0726511400001, abs=1e-10)
    assert float(final["vehicleCG[1]"]) == pytest.approx(
        -0.800270849913269,
        abs=1e-12,
    )
    assert float(final["vehicleCG[2]"]) == pytest.approx(
        1.511554237018939e-6,
        abs=1e-15,
    )
    assert float(final["vehicleCG[3]"]) == pytest.approx(
        0.2796156783777441,
        abs=1e-12,
    )


def test_vehicle_sim_qss_initialization_is_owned_by_experiment() -> None:
    repo_root = _repo_root()
    vehicle_sim = (
        repo_root / "BobLib" / "Experiments" / "Standards" / "VehicleSim.mo"
    ).read_text()
    axle = (
        repo_root / "BobLib" / "Chassis" / "Suspension" / "BaseClasses" / "PartialDoubleWishboneAxle.mo"
    ).read_text()
    required_vehicle_sim = (
        "chassisReferencePosition = pInitialization.chassisReferencePosition",
        "chassisReferenceAngles = pInitialization.chassisReferenceAngles",
        "fixInitialSuspensionAngles = true",
        "initialFrontLeftLowerArmAngle = pInitialization.frontLeftLowerArmAngle",
        "initialRearRightLowerArmAngle = pInitialization.rearRightLowerArmAngle",
    )
    assert all(fragment in vehicle_sim for fragment in required_vehicle_sim)
    assert "fixed = fixInitialLeftLowerArmAngle" in axle
    assert "fixed = fixInitialRightLowerArmAngle" in axle
    assert "fixed = fixInitialLowerArmAngularVelocities" in axle
    assert "frontSpringLength = 0.175156854188570" in vehicle_sim
    assert "rearSpringLength = 0.244911388707023" in vehicle_sim


def test_base_vehicle_sim_replaceable_placements_are_canonical() -> None:
    source_path = (
        _repo_root()
        / "BobLib"
        / "Experiments"
        / "Standards"
        / "BaseClasses"
        / "PartialVehicle.mo"
    )
    source = " ".join(source_path.read_text().split())
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
        "PartialVehicle replaceable constrained components must keep their "
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
        / "BaseClasses"
        / "PartialVehicle.mo"
    )
    source = " ".join(source_path.read_text().split())
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
        "PartialVehicle placements with an explicit origin must use local "
        "extents centered on that origin. Offset extents can reload with "
        "size discrepancies in OMEdit.\n"
        + "\n".join(f"line {line}: {fragment}" for line, fragment in bad_fragments)
    )


@pytest.mark.parametrize("relative_path", ROD_TO_LOWER_AXLE_FILES, ids=str)
def test_axles_preserve_rod_to_lower_attachment_selector(relative_path: Path) -> None:
    source_path = _repo_root() / relative_path
    source = " ".join(source_path.read_text().split())

    required_snippets = (
        "leftRodWishboneMount[3] = if pAxle.actuation.rodToLower then pAxle.geometry.lower_o else pAxle.geometry.upper_o",
        "r = pAxle.actuation.rodMount - leftRodWishboneMount",
        "r = mirrorXZ(pAxle.actuation.rodMount - leftRodWishboneMount)",
        "if pAxle.actuation.rodToLower then",
        "connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o)",
        "connect(toRightApex.frame_a, rightWishboneUprightLoop.lowerFrame_o)",
        "connect(toLeftApex.frame_a, leftWishboneUprightLoop.upperFrame_o)",
        "connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o)",
    )

    missing = [snippet for snippet in required_snippets if snippet not in source]
    assert not missing, (
        f"{relative_path} must keep pAxle.actuation.rodToLower wired through both the "
        "apex translation and upper/lower wishbone frame connections. Missing:\n"
        + "\n".join(missing)
    )
