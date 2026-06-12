#!/usr/bin/env python3
"""Generate the active BobLib vehicle package from ``Generation/vehicle.yml``.

`make sync-vehicle-yaml` copies the repo-root ``vehicle.yml`` into that staged
location before generation runs.

The legacy multi-variant helpers remain in this module for tests and ad-hoc
experiments, but the command-line entrypoint now builds the single active
vehicle architecture plus the matching ``VehicleSim`` and ``FourPostSim``
models.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from textwrap import dedent
from typing import Any, cast

REPO_ROOT = Path(__file__).resolve().parents[4]
GENERATION_DIR = Path(__file__).resolve().parent
SCRIPTS_DIR = GENERATION_DIR / "scripts"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


# =============================================================================
# Topology metadata
# =============================================================================


@dataclass(frozen=True)
class AxleTopology:
    key: str
    token: str
    front_model: str
    rear_model: str
    has_stabar: bool


TOPOLOGIES: dict[str, AxleTopology] = {
    "direct": AxleTopology(
        key="direct",
        token="DWDirect",
        front_model="BobLib.Vehicle.Chassis.Suspension.FrAxleDW_Direct",
        rear_model="BobLib.Vehicle.Chassis.Suspension.RrAxleDW_Direct",
        has_stabar=False,
    ),
    "bellcrank": AxleTopology(
        key="bellcrank",
        token="DWBC",
        front_model="BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC",
        rear_model="BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC",
        has_stabar=False,
    ),
    "bellcrank_stabar": AxleTopology(
        key="bellcrank_stabar",
        token="DWBCStabar",
        front_model="BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar",
        rear_model="BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar",
        has_stabar=True,
    ),
}

DEFAULT_TIRE_SLIP_MODEL = "TransientSlip"


@dataclass(frozen=True)
class VehicleVariant:
    front: AxleTopology
    rear: AxleTopology

    @property
    def variant_name(self) -> str:
        return f"{self.front.token}_{self.rear.token}"

    @property
    def record_name(self) -> str:
        return f"{self.variant_name}Record"

    @property
    def vehicle_model_name(self) -> str:
        return f"Vehicle_{self.variant_name}"


@dataclass(frozen=True)
class GeneratedPaths:
    build_dir: Path
    results_dir: Path
    source_vehicle_path: Path
    vehicle_model_path: Path
    sim_model_path: Path


@dataclass(frozen=True)
class RecordCheck:
    variant: VehicleVariant
    expected_class: str
    expected_file: Path
    exists: bool


# =============================================================================
# General helpers
# =============================================================================


def indent(text: str, spaces: int) -> str:
    pad = " " * spaces
    return "\n".join(pad + line if line else line for line in text.splitlines())


def parse_variant(front: str, rear: str) -> VehicleVariant:
    try:
        front_topology = TOPOLOGIES[front]
    except KeyError as exc:
        raise ValueError(
            f"Unknown front topology {front!r}. "
            f"Valid options: {sorted(TOPOLOGIES)}"
        ) from exc

    try:
        rear_topology = TOPOLOGIES[rear]
    except KeyError as exc:
        raise ValueError(
            f"Unknown rear topology {rear!r}. "
            f"Valid options: {sorted(TOPOLOGIES)}"
        ) from exc

    return VehicleVariant(front=front_topology, rear=rear_topology)


def all_variants() -> list[VehicleVariant]:
    return [
        parse_variant(front, rear)
        for front in TOPOLOGIES
        for rear in TOPOLOGIES
    ]


def default_generation_dir() -> Path:
    return Path(__file__).resolve().parent


def default_boblib_root() -> Path:
    """
    Assumes this file is at:

        <repo>/Generation/generate_vehicle_model.py

    and BobLib package.mo is at:

        <repo>/BobLib/package.mo
    """
    return Path(__file__).resolve().parent.parent / "BobLib"


# =============================================================================
# Record validation
# =============================================================================


def expected_record_path(*, variant: VehicleVariant, boblib_root: Path) -> Path:
    return boblib_root / "Resources" / "VehicleDefn" / f"{variant.record_name}.mo"


def check_record_exists(*, variant: VehicleVariant, boblib_root: Path) -> RecordCheck:
    record_path = expected_record_path(variant=variant, boblib_root=boblib_root)

    return RecordCheck(
        variant=variant,
        expected_class=f"BobLib.Resources.VehicleDefn.{variant.record_name}",
        expected_file=record_path,
        exists=record_path.exists(),
    )


def check_required_records(
    *,
    variants: list[VehicleVariant],
    boblib_root: Path,
) -> list[RecordCheck]:
    return [
        check_record_exists(variant=variant, boblib_root=boblib_root)
        for variant in variants
    ]


def print_record_report(checks: list[RecordCheck]) -> None:
    missing = [check for check in checks if not check.exists]
    present = [check for check in checks if check.exists]

    print("\nRecord preflight:")
    print(f"  Found:   {len(present)}")
    print(f"  Missing: {len(missing)}")

    if present:
        print("\n  Existing records:")
        for check in present:
            print(f"    ✅ {check.expected_class}")

    if missing:
        print("\n  Missing records:")
        for check in missing:
            print(f"    ❌ {check.expected_class}")
            print(f"       expected file: {check.expected_file}")

        print(
            "\nGenerated files will still be written, but any setup with a missing "
            "record will fail to translate until that record exists."
        )


# =============================================================================
# Vehicle wrapper renderer
# =============================================================================


def render_tire_redeclare(axle_prefix: str, side: str) -> str:
    """
    Render one tire redeclare.

    axle_prefix:
        "Fr" or "Rr"

    side:
        "left" or "right"
    """
    if axle_prefix not in {"Fr", "Rr"}:
        raise ValueError(f"axle_prefix must be 'Fr' or 'Rr', got {axle_prefix!r}")

    if side not in {"left", "right"}:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    instance = "leftTire" if side == "left" else "rightTire"
    tire_model = f"pVehicle.p{axle_prefix}TireModel"
    longitudinal_torque_sign = "-1"
    if DEFAULT_TIRE_SLIP_MODEL == "TransientSlip":
        slip_model = (
            "redeclare Tire.MF52.SlipModel.TransientSlip slipModel(\n"
            f"    FNOMIN = {tire_model}.relaxation.FNOMIN,\n"
            f"    UNLOADED_RADIUS = {tire_model}.relaxation.UNLOADED_RADIUS,\n"
            f"    LFZO = {tire_model}.relaxation.LFZO,\n"
            f"    PTX1 = {tire_model}.relaxation.PTX1,\n"
            f"    PTX2 = {tire_model}.relaxation.PTX2,\n"
            f"    PTX3 = {tire_model}.relaxation.PTX3,\n"
            f"    PTY1 = {tire_model}.relaxation.PTY1,\n"
            f"    PTY2 = {tire_model}.relaxation.PTY2,\n"
            f"    PKY3 = {tire_model}.relaxation.PKY3,\n"
            f"    LSGKP = {tire_model}.relaxation.LSGKP,\n"
            f"    LSGAL = {tire_model}.relaxation.LSGAL)"
        )
    else:
        slip_model = f"redeclare Tire.MF52.SlipModel.{DEFAULT_TIRE_SLIP_MODEL} slipModel"

    return dedent(
        f"""
        redeclare Tire.MF52Tire {instance}(
          pPartialWheel = pVehicle.p{axle_prefix}PartialWheel,
          pTireModel = {tire_model},
          redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
            longitudinalTorqueSign = {longitudinal_torque_sign},
            partialWheelParams = pVehicle.p{axle_prefix}PartialWheel,
            wheel1DOF_YParams = pVehicle.p{axle_prefix}Tire1DOF_YParams),
          {slip_model})
        """
    ).strip()


def render_axle_redeclare(
    *,
    instance_name: str,
    model_name: str,
    axle_prefix: str,
    has_stabar: bool,
) -> str:
    """
    Render one axle redeclare block inside Chassis_LockRrSteer.
    """
    stabar_line = (
        f"      pStabar = pVehicle.p{axle_prefix}Stabar,\n" if has_stabar else ""
    )

    left_tire = indent(render_tire_redeclare(axle_prefix, "left"), 6)
    right_tire = indent(render_tire_redeclare(axle_prefix, "right"), 6)

    return (
        f"    redeclare {model_name} {instance_name}(\n"
        f"      pAxle = pVehicle.p{axle_prefix}AxleDW,\n"
        f"      pRack = pVehicle.p{axle_prefix}Rack,\n"
        f"{stabar_line}"
        f"      pLeftPartialWheel = pVehicle.p{axle_prefix}PartialWheel,\n"
        f"      pLeftDW = pVehicle.p{axle_prefix}DW,\n"
        f"      pLeftAxleMass = pVehicle.p{axle_prefix}AxleMass,\n"
        f"{left_tire},\n"
        f"{right_tire})"
    )


def render_space_frame_redeclare(*, fr_ref: str, rr_ref: str) -> str:
    """Render the compliant chassis frame redeclare."""
    return dedent(
        """
        redeclare BobLib.Vehicle.Chassis.Body.FrameCompX spaceFrame(
          frRef = {fr_ref},
          rrRef = {rr_ref},
          pSprung = pVehicle.pSprungMass,
          pSprungMass = pVehicle.pSprungMass,
          torsionalStiff = pVehicle.pTorsionalStiff)
        """
    ).strip().format(fr_ref=fr_ref, rr_ref=rr_ref)


def render_vehicle_model(variant: VehicleVariant) -> str:
    fr_axle = render_axle_redeclare(
        instance_name="frAxleDW",
        model_name=variant.front.front_model,
        axle_prefix="Fr",
        has_stabar=variant.front.has_stabar,
    )

    rr_axle = render_axle_redeclare(
        instance_name="rrAxleDW",
        model_name=variant.rear.rear_model,
        axle_prefix="Rr",
        has_stabar=variant.rear.has_stabar,
    )

    chassis_params = ",\n".join(
        [
            indent(fr_axle.strip(), 6),
            indent(rr_axle.strip(), 6),
            indent(
                render_space_frame_redeclare(
                    fr_ref="{pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]}",
                    rr_ref="{pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]}",
                ),
                6,
            ),
        ]
    )

    return (
        f"within BobLib.Vehicle;\n\n"
        f"model {variant.vehicle_model_name}\n"
        "  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;\n"
        f"  import BobLib.Resources.VehicleDefn.{variant.record_name};\n\n"
        "  // Record parameters\n"
        f"  parameter {variant.record_name} pVehicle;\n\n"
        "  extends BobLib.Vehicle.VehicleBase(\n"
        "    pAero = pVehicle.pAero,\n"
        "    pSprungMass = pVehicle.pSprungMass,\n"
        "    pFrAxleMass = pVehicle.pFrAxleMass,\n"
        "    pRrAxleMass = pVehicle.pRrAxleMass,\n"
        "    redeclare BobLib.Vehicle.Chassis.Chassis_LockRrSteer chassis(\n"
        f"{chassis_params}\n"
        "    ));\n\n"
        "  Powertrain.PowertrainBatInvMotDiff ptn(\n"
        "    Ns = pVehicle.pPowertrain.Ns,\n"
        "    Np = pVehicle.pPowertrain.Np,\n"
        "    SOC_start = pVehicle.pPowertrain.SOC_start,\n"
        "    finalDriveRatio = pVehicle.pPowertrain.finalDriveRatio,\n"
        "    launch_w_eps = pVehicle.pPowertrain.launch_w_eps,\n"
        "    drivetrainAxis = pVehicle.pPowertrain.drivetrainAxis,\n"
        "    rMotorRotor = pVehicle.pPowertrain.rMotorRotor,\n"
        "    rDiffInputRotor = pVehicle.pPowertrain.rDiffInputRotor,\n"
        "    rDifferential = pVehicle.pPowertrain.rDifferential,\n"
        "    motorRotorJ = pVehicle.pPowertrain.motorRotorJ,\n"
        "    diffInputRotorJ = pVehicle.pPowertrain.diffInputRotorJ,\n"
        "    diff_use_lsd = pVehicle.pPowertrain.diff_use_lsd,\n"
        "    diff_driveSideTorqueSign = pVehicle.pPowertrain.diff_driveSideTorqueSign,\n"
        "    diff_T_preload = pVehicle.pPowertrain.diff_T_preload,\n"
        "    diff_lockFractionAccel = pVehicle.pPowertrain.diff_lockFractionAccel,\n"
        "    diff_lockFractionDecel = pVehicle.pPowertrain.diff_lockFractionDecel,\n"
        "    diff_T_capacity_max = pVehicle.pPowertrain.diff_T_capacity_max,\n"
        "    diff_clutchEffectiveRadius = pVehicle.pPowertrain.diff_clutchEffectiveRadius,\n"
        "    diff_kineticFrictionRatio = pVehicle.pPowertrain.diff_kineticFrictionRatio,\n"
        "    diff_w_transition = pVehicle.pPowertrain.diff_w_transition,\n"
        "    diff_c_viscous = pVehicle.pPowertrain.diff_c_viscous,\n"
        "    halfshaftLeftC = pVehicle.pPowertrain.halfshaftLeftC,\n"
        "    halfshaftLeftJEquivalent = pVehicle.pPowertrain.halfshaftLeftJEquivalent,\n"
        "    halfshaftLeftD = pVehicle.pPowertrain.halfshaftLeftD,\n"
        "    halfshaftRightC = pVehicle.pPowertrain.halfshaftRightC,\n"
        "    halfshaftRightJEquivalent = pVehicle.pPowertrain.halfshaftRightJEquivalent,\n"
        "    halfshaftRightD = pVehicle.pPowertrain.halfshaftRightD,\n"
        "    motorVdcMax = pVehicle.pPowertrain.motorVdcMax,\n"
        "    motorRpmMaxPeak = pVehicle.pPowertrain.motorRpmMaxPeak,\n"
        "    motorTPeak = pVehicle.pPowertrain.motorTPeak,\n"
        "    motorTCont = pVehicle.pPowertrain.motorTCont,\n"
        "    motorIPeak = pVehicle.pPowertrain.motorIPeak,\n"
        "    motorICont = pVehicle.pPowertrain.motorICont,\n"
        "    motorKtNmPerA = pVehicle.pPowertrain.motorKtNmPerA,\n"
        "    motorPeakTime = pVehicle.pPowertrain.motorPeakTime,\n"
        "    motorPMechPeak = pVehicle.pPowertrain.motorPMechPeak,\n"
        "    motorPContLow = pVehicle.pPowertrain.motorPContLow,\n"
        "    motorPContHigh = pVehicle.pPowertrain.motorPContHigh,\n"
        "    motorEtaMot = pVehicle.pPowertrain.motorEtaMot,\n"
        "    motorEtaReg = pVehicle.pPowertrain.motorEtaReg) annotation(\n"
        "    Placement(transformation(origin = {0, -70}, extent = {{-20, -20}, {20, 20}})));\n\n"
        "  replaceable Electronics.ElectronicsAssembly elc(\n"
        "    tau_max = pVehicle.pPowertrain.tau_max,\n"
        "    w_eps = pVehicle.pPowertrain.launch_w_eps,\n"
        "    motorSpeedSign = pVehicle.pPowertrain.vcuMotorSpeedSign,\n"
        "    inverterPMaxMot = pVehicle.pPowertrain.inverterPMaxMot,\n"
        "    inverterPMaxReg = pVehicle.pPowertrain.inverterPMaxReg,\n"
        "    inverterVdcMax = pVehicle.pPowertrain.inverterVdcMax)\n"
        "    constrainedby Electronics.ElectronicsBase annotation(\n"
        "    Placement(transformation(origin = {-58, -82}, extent = {{-10, -10}, {10, 10}})));\n\n"
        "  Modelica.Blocks.Sources.BooleanConstant inverterEnabled(k = true) annotation(\n"
        "    Placement(transformation(origin = {-96, -86}, extent = {{-8, -8}, {8, 8}})));\n\n"
        "  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(\n"
        "    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}),\n"
        "    iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));\n\n"
        "  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(\n"
        "    Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}),\n"
        "    iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));\n\n"
        "  Modelica.Blocks.Interfaces.RealInput uPTNTorque\n"
        "    \"Requested total rear-axle drive torque [Nm]\" annotation(\n"
        "    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),\n"
        "    iconTransformation(origin = {-30, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));\n\n"
        "  Modelica.Blocks.Interfaces.RealInput uPTNRegenLimit annotation(\n"
        "    Placement(transformation(origin = {50, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),\n"
        "    iconTransformation(origin = {30, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));\n\n"
        "equation\n"
        "  connect(chassis.rrAxleFrame, ptn.mountFrame) annotation(\n"
        "    Line(points = {{0, -40}, {0, -50}}, color = {95, 95, 95}));\n\n"
        "  connect(ptn.leftFlange, chassis.flangeRL) annotation(\n"
        "    Line(points = {{-20, -78}, {-70, -78}, {-70, -40}, {-58, -40}}));\n\n"
        "  connect(ptn.rightFlange, chassis.flangeRR) annotation(\n"
        "    Line(points = {{20, -78}, {70, -78}, {70, -40}, {58, -40}}));\n\n"
        "  connect(flangeFL, chassis.flangeFL) annotation(\n"
        "    Line(points = {{-100, 60}, {-80, 60}, {-80, 38}, {-58, 38}}));\n\n"
        "  connect(flangeFR, chassis.flangeFR) annotation(\n"
        "    Line(points = {{100, 60}, {80, 60}, {80, 38}, {58, 38}}));\n\n"
        "  elc.cmd_torque_motor = uPTNTorque / pVehicle.pPowertrain.finalDriveRatio;\n"
        "  elc.sens_motor_speed = ptn.motorSpeed;\n\n"
        "  connect(uPTNRegenLimit, elc.cmd_regen_limit) annotation(\n"
        "    Line(points = {{50, -120}, {50, -86}, {-70, -86}}, color = {0, 0, 127}));\n\n"
        "  connect(inverterEnabled.y, elc.cmd_inverter_enable) annotation(\n"
        "    Line(points = {{-87, -86}, {-79.5, -86}, {-79.5, -82}, {-70, -82}}, color = {255, 0, 255}));\n\n"
        "  connect(elc.P_motor, ptn.P_elec) annotation(\n"
        "    Line(points = {{-46, -82}, {-28, -82}, {-28, -94}, {-12, -94}}, color = {0, 0, 127}));\n\n"
        "  connect(elc.p, ptn.hv_p) annotation(\n"
        "    Line(points = {{-68, -78}, {-78, -78}, {-78, -62}, {-20, -62}}, color = {0, 0, 255}));\n\n"
        "  connect(elc.n, ptn.hv_n) annotation(\n"
        "    Line(points = {{-68, -86}, {-84, -86}, {-84, -68}, {-20, -68}}, color = {0, 0, 255}));\n\n"
        "annotation(\n"
        "  Diagram(graphics),\n"
        "  Icon(graphics = {\n"
        "    Line(origin = {-130, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),\n"
        "    Line(origin = {190, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),\n"
        "    Line(origin = {-130, -120}, points = {{-10, 0}, {-30, 0}}, pattern = LinePattern.Dash, thickness = 1),\n"
        "    Line(origin = {190, -120}, points = {{-30, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),\n"
        "    Line(origin = {-80, -159}, points = {{80, -41}, {80, -31}, {-80, -31}, {-80, 39}}, color = {0, 0, 255}),\n"
        "    Line(origin = {71.18, -171.82}, points = {{-71.1799, -18.1799}, {88.8201, -18.1799}, {88.8201, 51.8201}}, color = {0, 0, 255})\n"
        "  }));\n"
        f"end {variant.vehicle_model_name};\n"
    )


# =============================================================================
# VehicleSim renderer
# =============================================================================


def render_vehicle_sim(variant: VehicleVariant) -> str:
    """
    Render the replacement Standards.VehicleSim.

    This intentionally always emits:

        within BobLib.Standards;
        model VehicleSim
          ...
        end VehicleSim;

    The variant-specific part is the imported record and instantiated vehicle.
    """
    import build_vehicle_sim as build_vehicle

    canonical_data = {
        "output": {"sim_package": "BobLib.Standards"},
        "architecture": {"vehicle_model": variant.vehicle_model_name},
    }
    return build_vehicle.render_vehicle_sim(
        data=cast(dict[str, Any], canonical_data),
        record_name=variant.record_name,
    )

# =============================================================================
# File writing
# =============================================================================


def write_variant(
    *,
    variant: VehicleVariant,
    generation_dir: Path,
    boblib_root: Path,
    overwrite: bool = True,
) -> GeneratedPaths:
    builds_root = generation_dir / "Builds"
    results_root = generation_dir / "Results"

    build_dir = builds_root / variant.variant_name
    results_dir = results_root / variant.variant_name

    build_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / ".gitkeep").touch(exist_ok=True)

    source_vehicle_path = boblib_root / "Vehicle" / f"{variant.vehicle_model_name}.mo"
    vehicle_model_path = build_dir / f"{variant.vehicle_model_name}.mo"
    sim_model_path = build_dir / "VehicleSim.mo"

    files = {
        source_vehicle_path: render_vehicle_model(variant),
        vehicle_model_path: render_vehicle_model(variant),
        sim_model_path: render_vehicle_sim(variant),
    }

    for path, contents in files.items():
        if path.exists() and not overwrite:
            raise FileExistsError(
                f"{path} already exists. Set overwrite=True to replace generated files."
            )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    return GeneratedPaths(
        build_dir=build_dir,
        results_dir=results_dir,
        source_vehicle_path=source_vehicle_path,
        vehicle_model_path=vehicle_model_path,
        sim_model_path=sim_model_path,
    )


def generate_one(
    *,
    front: str,
    rear: str,
    generation_dir: Path | None = None,
    boblib_root: Path | None = None,
    overwrite: bool = True,
) -> GeneratedPaths:
    generation_dir = generation_dir or default_generation_dir()
    boblib_root = boblib_root or default_boblib_root()

    variant = parse_variant(front, rear)

    record_check = check_record_exists(
        variant=variant,
        boblib_root=boblib_root,
    )
    print_record_report([record_check])

    return write_variant(
        variant=variant,
        generation_dir=generation_dir,
        boblib_root=boblib_root,
        overwrite=overwrite,
    )


def generate_all(
    *,
    generation_dir: Path | None = None,
    boblib_root: Path | None = None,
    overwrite: bool = True,
) -> list[GeneratedPaths]:
    generation_dir = generation_dir or default_generation_dir()
    boblib_root = boblib_root or default_boblib_root()

    variants = all_variants()

    record_checks = check_required_records(
        variants=variants,
        boblib_root=boblib_root,
    )
    print_record_report(record_checks)

    outputs: list[GeneratedPaths] = []

    for variant in variants:
        outputs.append(
            write_variant(
                variant=variant,
                generation_dir=generation_dir,
                boblib_root=boblib_root,
                overwrite=overwrite,
            )
        )

    return outputs


# =============================================================================
# Main
# =============================================================================


def generate_active_package(*, overwrite: bool = True) -> None:
    from build_common import (
        active_vehicle_variant,
        boblib_root,
        load_yaml,
        record_name_from_yaml,
        vehicle_model_name,
        vehicle_yaml_path,
    )
    from build_four_post_sim import build_four_post_sim
    from build_vehicle_sim import build_vehicle_sim

    source_yaml = vehicle_yaml_path()
    data = load_yaml(source_yaml)
    variant = active_vehicle_variant(data, source_yaml)
    record_name = record_name_from_yaml(data, source_yaml)
    vehicle_model = vehicle_model_name(data, record_name)
    vehicle_root = boblib_root(data)
    record_path = vehicle_root / "Resources" / "VehicleDefn" / f"{record_name}.mo"
    vehicle_model_path = vehicle_root / "Vehicle" / f"{vehicle_model}.mo"

    vehicle_sim_path = build_vehicle_sim(source_yaml=source_yaml, overwrite=overwrite)
    four_post_record_path, four_post_model_path = build_four_post_sim(
        source_yaml=source_yaml,
        overwrite=overwrite,
    )

    print(f"\nGenerated active BobLib package from {source_yaml}")
    print(f"  Active topology:     {variant.variant_name}")
    print(f"  Vehicle record:      {record_path}")
    print(f"  Vehicle model:       {vehicle_model_path}")
    print(f"  VehicleSim:          {vehicle_sim_path}")
    print(f"  FourPostEval record: {four_post_record_path}")
    print(f"  FourPostSim:         {four_post_model_path}")


def main() -> None:
    generate_active_package(overwrite=True)


if __name__ == "__main__":
    main()
