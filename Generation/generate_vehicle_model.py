#!/usr/bin/env python3
"""
Generate concrete BobLib vehicle and simulation Modelica files.

Expected layout:

    Generation/
      generate_vehicle_model.py
      Builds/
      Results/

Running this script with no arguments generates all 9 structural setups and
overwrites existing generated files:

    python Generation/generate_vehicle_model.py

A generated variant creates:

    Generation/Builds/<Variant>/
      Vehicle_<Variant>.mo
      VehicleSim.mo

    Generation/Results/<Variant>/

Important convention:

    Vehicle_<Variant>.mo keeps its variant-specific model name.

    VehicleSim.mo is intentionally replaced per generated build and always
    contains:

        within BobLib.Standards;
        model VehicleSim
          ...
        end VehicleSim;

This matches the existing BobLib workflow where Standards.VehicleSim is the
simulation entrypoint, but the vehicle wrapper underneath is variant-specific.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from textwrap import dedent


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

        <repo>/package.mo
    """
    return Path(__file__).resolve().parent.parent


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

    return dedent(
        f"""
        redeclare Tire.MF52Tire {instance}(
          pPartialWheel = pVehicle.p{axle_prefix}PartialWheel,
          pTireModel = pVehicle.p{axle_prefix}TireModel,
          redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
            partialWheelParams = pVehicle.p{axle_prefix}PartialWheel,
            wheel1DOF_YParams = pVehicle.p{axle_prefix}Tire1DOF_YParams),
          redeclare Tire.MF52.SlipModel.TransientSlip slipModel)
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
    """Render the compliant body frame redeclare for the chassis."""
    return dedent(
        """
        redeclare BobLib.Vehicle.Chassis.Body.FrameCompX spaceFrame(
          frRef = {fr_ref},
          rrRef = {rr_ref},
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
            "      pSprungMass = pVehicle.pSprungMass",
            "      pVehicleCG = pVehicleCG",
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
        "  Powertrain.PTNPlaceholder ptnPlaceholder annotation(\n"
        "    Placement(transformation(origin = {0, -70}, extent = {{-20, -4}, {20, 4}})));\n\n"
        "  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(\n"
        "    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}),\n"
        "    iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));\n\n"
        "  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(\n"
        "    Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}),\n"
        "    iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));\n\n"
        "  Modelica.Blocks.Interfaces.RealInput uPTNTorque annotation(\n"
        "    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),\n"
        "    iconTransformation(origin = {0, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));\n\n"
        "equation\n"
        "  connect(chassis.rrAxleFrame, ptnPlaceholder.mountFrame) annotation(\n"
        "    Line(points = {{0, -40}, {0, -66}}, color = {95, 95, 95}));\n\n"
        "  connect(ptnPlaceholder.leftFlange, chassis.flangeRL) annotation(\n"
        "    Line(points = {{-20, -70}, {-70, -70}, {-70, -40}, {-58, -40}}));\n\n"
        "  connect(ptnPlaceholder.rightFlange, chassis.flangeRR) annotation(\n"
        "    Line(points = {{20, -70}, {70, -70}, {70, -40}, {58, -40}}));\n\n"
        "  connect(flangeFL, chassis.flangeFL) annotation(\n"
        "    Line(points = {{-100, 60}, {-80, 60}, {-80, 38}, {-58, 38}}));\n\n"
        "  connect(flangeFR, chassis.flangeFR) annotation(\n"
        "    Line(points = {{100, 60}, {80, 60}, {80, 38}, {58, 38}}));\n\n"
        "  connect(uPTNTorque, ptnPlaceholder.u) annotation(\n"
        "    Line(points = {{0, -120}, {0, -78}}, color = {0, 0, 127}));\n\n"
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
    return dedent(
        f"""
        within BobLib.Standards;

        model VehicleSim
          import Modelica.SIunits;
          import Modelica.Constants.pi;
          import Modelica.Math.Vectors.norm;
          import Modelica.Mechanics.MultiBody.Frames;
          import BobLib.Utilities.Math.Vector;

          // Import vehicle records
          import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

          import BobLib.Resources.VehicleDefn.{variant.record_name};

          inner parameter SIunits.Length linkDiameter = 0.020;
          inner parameter SIunits.Length jointDiameter = 0.030;

          parameter {variant.record_name} pVehicle;

          parameter Integer useMode = 0
            "0 - closed-loop radius and velocity; 1 - open-loop sinusoidal steer, constant velocity; 2 - custom open-loop steer and drive torque"
            annotation(Evaluate = false);

          // Toggle controllers
          final parameter Boolean closedLoopRadius = useMode == 0;
          final parameter Boolean closedLoopVelocity = useMode == 0 or useMode == 1 or useMode == 2;

          parameter Modelica.SIunits.Time steerStart = 1.0
            "Start time"
            annotation(Evaluate = false);

          // Closed-loop parameters
          parameter SIunits.Length targetRad = 20
            "Target maneuver curvature"
            annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

          parameter SIunits.Velocity targetVel = 15
            "Target maneuver velocity"
            annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

          parameter SIunits.Velocity initialVel = targetVel
            "Initial velocity"
            annotation(Evaluate = false);

          parameter Real curvGain = 3
            "Proportional gain of curvature controller"
            annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

          parameter Real curvTi = 0.02
            "Time constant of curvature controller"
            annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

          parameter Real velGain = 200
            "Proportional gain of velocity controller"
            annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

          parameter Real velTi = 1
            "Time constant of velocity controller"
            annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

          parameter Real radErrorTol = 0.002
            "SteadyStateEval radius error tolerance"
            annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

          parameter Real der_radErrorTol = 0.5
            "SteadyStateEval radius error derivative tolerance"
            annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

          parameter Real der_yawVelTol = 0.01;

          // Ramp-steer parameters
          parameter SIunits.Angle frRampSteerHeight = 5*pi/180
            "Ramp steer target angle";

          parameter SIunits.Time frRampSteerDuration = 0.001
            "Ramp steer duration";

          // Frequency response parameters
          parameter SIunits.Angle steerAmp = 6*pi/180
            "Amplitude"
            annotation(Evaluate = false);

          parameter SIunits.Frequency steerFreq = 1.0
            "Frequency (Hz)"
            annotation(Evaluate = false);

          // Raw signal parameters
          Real frSteerCmd;
          Real driveTorqueCmd;
          Real bodyVels[3];
          Real bodyAccels[3];
          Real bodyAngles[3];
          Real curvature;
          Real speed;
          Real curvError;
          Real radError;
          Real velError;
          Real steerSine;
          Real steerRamp;

          // Standard outputs
          SIunits.Acceleration accX;
          SIunits.Acceleration accY;
          SIunits.Angle handwheelAngle;
          SIunits.Torque handwheelTorque;
          SIunits.Angle leftSteerAngle;
          SIunits.Angle rightSteerAngle;
          SIunits.Angle roll;
          SIunits.Angle sideslip;
          SIunits.Velocity velX;
          SIunits.Velocity velY;
          SIunits.AngularVelocity yawVel;

          inner Modelica.Mechanics.MultiBody.World world(n = {{0, 0, -1}}) annotation(
            Placement(transformation(origin = {{-130, -110}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          // Vehicle
          BobLib.Vehicle.{variant.vehicle_model_name} vehicle(
            pVehicle = pVehicle) annotation(
            Placement(transformation(origin = {{0, 20}}, extent = {{{{-45, -50}}, {{45, 50}}}})));

          // CG motion
          Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
            animation = false,
            r_rel_a(start = {{0, 0, 0}}, each fixed = true),
            enforceStates = false,
            v_rel_a(start = {{initialVel, 0, 0}}, each fixed = true)) annotation(
            Placement(transformation(origin = {{100, 90}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          // Front steer position
          Modelica.Mechanics.Rotational.Sources.Position frSteerPosition annotation(
            Placement(transformation(origin = {{-30, 110}}, extent = {{{{-10, -10}}, {{10, 10}}}}, rotation = -0)));

          // Body attitude sensor
          Modelica.Mechanics.MultiBody.Sensors.RelativeAngles sprungAngles annotation(
            Placement(transformation(origin = {{70, -70}}, extent = {{{{-10, -10}}, {{10, 10}}}}, rotation = 90)));

          // Calculated parameters
          final parameter Real cpInitFL[3] =
            pVehicle.pFrDW.wheelCenter +
            Frames.resolve1(
              Frames.axesRotations(
                {{1, 2, 3}},
                {{
                  pVehicle.pFrPartialWheel.staticGamma*pi/180,
                  0,
                  pVehicle.pFrPartialWheel.staticAlpha*pi/180
                }},
                {{0, 0, 0}}),
              {{0, 0, -pVehicle.pFrPartialWheel.R0}});

          final parameter Real cpInitFR[3] = Vector.mirrorXZ(cpInitFL);

          final parameter Real cpInitRL[3] =
            pVehicle.pRrDW.wheelCenter +
            Frames.resolve1(
              Frames.axesRotations(
                {{1, 2, 3}},
                {{
                  pVehicle.pRrPartialWheel.staticGamma*pi/180,
                  0,
                  pVehicle.pRrPartialWheel.staticAlpha*pi/180
                }},
                {{0, 0, 0}}),
              {{0, 0, -pVehicle.pRrPartialWheel.R0}});

          final parameter Real cpInitRR[3] = Vector.mirrorXZ(cpInitRL);

        protected
          // QSS detection variables
          discrete Real t_curv_hit(start = -1);
          discrete Real t_yawVel_hit(start = -1);

          Real leftWheelVector[3];
          Real rightWheelVector[3];

          // Initial geometry
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedFL(
            r = cpInitFL,
            animation = false) annotation(
            Placement(transformation(origin = {{-130, 10}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          Modelica.Mechanics.MultiBody.Parts.Fixed fixedFR(
            r = cpInitFR,
            animation = false) annotation(
            Placement(transformation(origin = {{130, 10}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          Modelica.Mechanics.MultiBody.Parts.Fixed fixedRL(
            r = cpInitRL,
            animation = false) annotation(
            Placement(transformation(origin = {{-130, -50}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          Modelica.Mechanics.MultiBody.Parts.Fixed fixedRR(
            r = cpInitRR,
            animation = false) annotation(
            Placement(transformation(origin = {{130, -50}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
            r = vehicle.pVehicleCG,
            animation = false) annotation(
            Placement(transformation(origin = {{130, 90}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          // Ground interface
          BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
            Placement(transformation(origin = {{-100, 10}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
            Placement(transformation(origin = {{100, 10}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
            Placement(transformation(origin = {{-100, -50}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
            Placement(transformation(origin = {{100, -50}}, extent = {{{{10, -10}}, {{-10, 10}}}})));

          // Curvature controller
          Modelica.Blocks.Sources.RealExpression curvErrorExpression(
            y = curvError) annotation(
            Placement(transformation(origin = {{-110, 110}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          Modelica.Blocks.Continuous.PI curvPI(
            k = curvGain,
            T = curvTi,
            initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
            Placement(transformation(origin = {{-70, 110}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          // Speed Controller
          Modelica.Blocks.Sources.RealExpression velErrorExpression(
            y = velError) annotation(
            Placement(transformation(origin = {{-70, -50}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

          Modelica.Blocks.Continuous.PI speedPI(
            k = velGain,
            T = velTi) annotation(
            Placement(transformation(origin = {{-30, -50}}, extent = {{{{-10, -10}}, {{10, 10}}}})));

        initial equation
          vehicle.chassis.frAxleDW.leftTire.wheelModel.hubAxis.w =
            initialVel / pVehicle.pFrPartialWheel.R0;

          vehicle.chassis.frAxleDW.rightTire.wheelModel.hubAxis.w =
            initialVel / pVehicle.pFrPartialWheel.R0;

          vehicle.chassis.rrAxleDW.leftTire.wheelModel.hubAxis.w =
            initialVel / pVehicle.pRrPartialWheel.R0;

          vehicle.chassis.rrAxleDW.rightTire.wheelModel.hubAxis.w =
            initialVel / pVehicle.pRrPartialWheel.R0;

        equation
          // Curvature quantities
          curvature =
            vehicle.chassis.spaceFrame.sprungBody.w_a[3] / max(speed, 0.1);

          // SteadyStateEval-style radius error
          radError =
            abs(speed / max(abs(vehicle.chassis.spaceFrame.sprungBody.w_a[3]), 0.1)
            - abs(targetRad));

          // SteadyStateEval-style QSS detection, only active for useMode == 0
          when useMode == 0
             and abs(radError) < radErrorTol
             and abs(der(radError)) < der_radErrorTol
             and pre(t_curv_hit) < 0 then
            t_curv_hit = time;
          end when;

          when useMode == 0
             and t_curv_hit > 0
             and time > t_curv_hit + 0.1 then
            terminate("Reached steady-state (held 0.1s)");
          end when;

          // Ramp-steer steady-state detection
          when useMode == 2
             and time > steerStart + frRampSteerDuration
             and abs(der(yawVel)) < der_yawVelTol
             and pre(t_yawVel_hit) < 0 then
            t_yawVel_hit = time;

          elsewhen useMode == 2
             and abs(der(yawVel)) >= der_yawVelTol then
            t_yawVel_hit = -1;
          end when;

          when useMode == 2
             and t_yawVel_hit > 0
             and time > t_yawVel_hit + 0.1 then
            terminate("Reached ramp-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
          end when;

          // SteadyStateEval curvature controller for useMode == 0.
          // Intentionally matches old SteadyStateEval: ramp from t = 1.0 to t = 1.2.
          curvError =
            if useMode == 0 then
              smooth(1, min(1, max(0, (time - 1)/0.2))) *
              (1/targetRad - vehicle.chassis.spaceFrame.sprungBody.w_a[3]/max(speed, 0.1))
            else
              0;

          // Sinusoidal steering profile for useMode == 1
          steerSine =
            if noEvent(useMode == 1 and time > steerStart) then
              steerAmp*sin(2*pi*steerFreq*(time - steerStart))
            else
              0;

          // Ramp-steer profile for useMode == 2
          steerRamp =
            frRampSteerHeight *
            noEvent(min(1, max(0, (time - steerStart) / frRampSteerDuration)));

          // Mode switching logic
          frSteerCmd =
            if useMode == 0 then
              curvPI.y
            elseif useMode == 1 then
              steerSine
            elseif useMode == 2 then
              steerRamp
            else
              0;

          driveTorqueCmd =
            if useMode == 0 or useMode == 1 or useMode == 2 then
              speedPI.y
            else
              0;

          // Apply steer and drive torque
          frSteerPosition.phi_ref = frSteerCmd;
          vehicle.uPTNTorque = driveTorqueCmd;

          // General quantities
          bodyVels =
            Frames.resolve2(
              vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
              vehicle.chassis.spaceFrame.sprungBody.v_0);

          bodyAccels =
            Frames.resolve2(
              vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
              vehicle.chassis.spaceFrame.sprungBody.a_0);

          bodyAngles =
            Frames.resolve2(
              vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
              sprungAngles.angles);

          leftWheelVector =
            Frames.resolve1(
              vehicle.chassis.frAxleFrame.R,
              Frames.resolve2(vehicle.frameFL.R, {{1, 0, 0}}));

          rightWheelVector =
            Frames.resolve1(
              vehicle.chassis.frAxleFrame.R,
              Frames.resolve2(vehicle.frameFR.R, {{1, 0, 0}}));

          leftSteerAngle = -1*atan(leftWheelVector[2] / leftWheelVector[1]);
          rightSteerAngle = -1*atan(rightWheelVector[2] / rightWheelVector[1]);

          handwheelAngle = vehicle.steerFlange.phi;

          // Speed quantities
          speed = norm(bodyVels);

          velError = targetVel - speed;

          // Kinematics
          velX = bodyVels[1];
          velY = bodyVels[2];
          yawVel = vehicle.chassis.spaceFrame.sprungBody.w_a[3];
          sideslip = atan(velY / velX);

          // Accelerations
          accX = bodyAccels[1];
          accY = bodyAccels[2];

          // Vehicle response
          roll = bodyAngles[1];

          // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
          handwheelTorque = -1*vehicle.steerFlange.tau;

          connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
            Line(points = {{{{120, 90}}, {{110, 90}}}}, color = {{95, 95, 95}}));

          connect(velErrorExpression.y, speedPI.u) annotation(
            Line(points = {{{{-59, -50}}, {{-42, -50}}}}, color = {{0, 0, 127}}));

          connect(fixedFL.frame_b, groundFL.frame_a) annotation(
            Line(points = {{{{-120, 10}}, {{-110, 10}}}}, color = {{95, 95, 95}}));

          connect(fixedFR.frame_b, groundFR.frame_a) annotation(
            Line(points = {{{{120, 10}}, {{110, 10}}}}, color = {{95, 95, 95}}));

          connect(fixedRL.frame_b, groundRL.frame_a) annotation(
            Line(points = {{{{-120, -50}}, {{-110, -50}}}}, color = {{95, 95, 95}}));

          connect(fixedRR.frame_b, groundRR.frame_a) annotation(
            Line(points = {{{{120, -50}}, {{110, -50}}}}, color = {{95, 95, 95}}));

          connect(curvErrorExpression.y, curvPI.u) annotation(
            Line(points = {{{{-99, 110}}, {{-82, 110}}}}, color = {{0, 0, 127}}));

          connect(vehicle.frameRL, groundRL.frame_b) annotation(
            Line(points = {{{{-44, -22}}, {{-100, -22}}, {{-100, -40}}}}, color = {{95, 95, 95}}));

          connect(groundRR.frame_b, vehicle.frameRR) annotation(
            Line(points = {{{{100, -40}}, {{100, -22}}, {{46, -22}}}}, color = {{95, 95, 95}}));

          connect(vehicle.frameFL, groundFL.frame_b) annotation(
            Line(points = {{{{-44, 38}}, {{-100, 38}}, {{-100, 20}}}}, color = {{95, 95, 95}}));

          connect(vehicle.frameFR, groundFR.frame_b) annotation(
            Line(points = {{{{46, 38}}, {{100, 38}}, {{100, 20}}}}, color = {{95, 95, 95}}));

          connect(frSteerPosition.flange, vehicle.steerFlange) annotation(
            Line(points = {{{{-20, 110}}, {{0, 110}}, {{0, 62}}, {{0, 62}}}}));

          connect(cgFreeMotion.frame_b, vehicle.cgFrame) annotation(
            Line(points = {{{{90, 90}}, {{70, 90}}, {{70, 20}}, {{46, 20}}}}, color = {{95, 95, 95}}));

          connect(world.frame_b, sprungAngles.frame_a) annotation(
            Line(points = {{{{-120, -110}}, {{70, -110}}, {{70, -80}}}}, color = {{95, 95, 95}}));

          connect(vehicle.cgFrame, sprungAngles.frame_b) annotation(
            Line(points = {{{{46, 20}}, {{70, 20}}, {{70, -60}}}}, color = {{95, 95, 95}}));

          annotation(
            Diagram(coordinateSystem(extent = {{{{-140, -120}}, {{140, 120}}}})),
            Icon(coordinateSystem(extent = {{{{-140, -120}}, {{140, 120}}}})),
            experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
            __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000",
            __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", noEquidistantTimeGrid = "()", s = "dassl", variableFilter = ".*"));

        end VehicleSim;
        """
    ).strip() + "\n"


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


def main() -> None:
    """
    Default behavior:

      - Generate all 9 structural topology combinations.
      - Overwrite existing generated files.
      - Use Generation/Builds and Generation/Results.
      - Assume package.mo lives one level above Generation/.
      - Do not generate README files.
      - Do not generate .mos scripts.
    """
    outputs = generate_all(overwrite=True)

    print("\nGenerated BobLib vehicle builds:")
    for output in outputs:
        print(f"  {output.build_dir.name}")
        print(f"    Build dir:     {output.build_dir}")
        print(f"    Results dir:   {output.results_dir}")
        print(f"    Vehicle model: {output.vehicle_model_path}")
        print(f"    VehicleSim:    {output.sim_model_path}")


if __name__ == "__main__":
    main()
