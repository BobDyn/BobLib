within BobLib.Chassis;

model DoubleWishbone

  "Detailed BobLib chassis configured by replaceable double-wishbone axle variants"

  import Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;
  import BobLib.Utilities.Math.Vector;

  replaceable record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord.ChassisRecord
    constrainedby BobLib.Records.VehicleRecord.Chassis.ChassisRecord annotation(choicesAllMatching = true);
  replaceable model FrAxleModel =
    BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar
    constrainedby BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle
    annotation(choicesAllMatching = true, Dialog(tab = "Plant Models", group = "Front axle"));
  replaceable model RrAxleModel =
    BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar
    constrainedby BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle
    annotation(choicesAllMatching = true, Dialog(tab = "Plant Models", group = "Rear axle"));
  replaceable model FrameModel =
    BobLib.Chassis.Body.FrameCompX
    constrainedby BobLib.Chassis.Body.FrameBase
    annotation(choicesAllMatching = true, Dialog(tab = "Plant Models", group = "Sprung frame"));

  parameter ChassisRecord pChassis
    "Chassis calibration: frame, front axle and rear axle" annotation(Dialog(group = "Calibration"));

  parameter Boolean fixInitialSuspensionAngles = false
    "Initialize suspension states from four-post-solved lower-arm angles" annotation(Dialog(group = "Initialization"), choices(checkBox = true));
  parameter SI.Angle initialFrontLeftLowerArmAngle = 0 "Front-left lower wishbone angle used for suspension initialization, in radians" annotation(Dialog(group = "Initialization"));
  parameter SI.Angle initialFrontRightLowerArmAngle = 0 "Front-right lower wishbone angle used for suspension initialization, in radians" annotation(Dialog(group = "Initialization"));
  parameter SI.Angle initialRearLeftLowerArmAngle = 0 "Rear-left lower wishbone angle used for suspension initialization, in radians" annotation(Dialog(group = "Initialization"));
  parameter SI.Angle initialRearRightLowerArmAngle = 0 "Rear-right lower wishbone angle used for suspension initialization, in radians" annotation(Dialog(group = "Initialization"));

  final parameter MassRecord pVehicleMass(
    m(fixed = false), rCM(each fixed = false), inertia(each fixed = false))
    "Complete vehicle mass properties including both axle sides";

  final parameter SI.Mass pTotalMass = pVehicleMass.m
    "Complete vehicle mass";

  final parameter SI.Position pVehicleCG[3] = pVehicleMass.rCM
    "Complete vehicle center of gravity";

  final parameter SI.Mass pInitializationMass =
    pChassis.frame.mass.m +
    pChassis.frontAxle.masses.unsprungMass.m +
    pChassis.frontAxle.masses.ucaMass.m +
    pChassis.frontAxle.masses.lcaMass.m +
    pChassis.frontAxle.masses.tieMass.m +
    pChassis.rearAxle.masses.unsprungMass.m +
    pChassis.rearAxle.masses.ucaMass.m +
    pChassis.rearAxle.masses.lcaMass.m +
    pChassis.rearAxle.masses.tieMass.m
    "Mass used only by the legacy start-pose estimate";

  final parameter SI.Position pInitializationCG[3] = {
    (
      pChassis.frame.mass.m*pChassis.frame.mass.rCM[1] +
      pChassis.frontAxle.masses.unsprungMass.m*pChassis.frontAxle.masses.unsprungMass.rCM[1] +
      pChassis.frontAxle.masses.ucaMass.m*pChassis.frontAxle.masses.ucaMass.rCM[1] +
      pChassis.frontAxle.masses.lcaMass.m*pChassis.frontAxle.masses.lcaMass.rCM[1] +
      pChassis.frontAxle.masses.tieMass.m*pChassis.frontAxle.masses.tieMass.rCM[1] +
      pChassis.rearAxle.masses.unsprungMass.m*pChassis.rearAxle.masses.unsprungMass.rCM[1] +
      pChassis.rearAxle.masses.ucaMass.m*pChassis.rearAxle.masses.ucaMass.rCM[1] +
      pChassis.rearAxle.masses.lcaMass.m*pChassis.rearAxle.masses.lcaMass.rCM[1] +
      pChassis.rearAxle.masses.tieMass.m*pChassis.rearAxle.masses.tieMass.rCM[1]
    )/pInitializationMass,
    (
      pChassis.frame.mass.m*pChassis.frame.mass.rCM[2] +
      pChassis.frontAxle.masses.unsprungMass.m*pChassis.frontAxle.masses.unsprungMass.rCM[2] +
      pChassis.frontAxle.masses.ucaMass.m*pChassis.frontAxle.masses.ucaMass.rCM[2] +
      pChassis.frontAxle.masses.lcaMass.m*pChassis.frontAxle.masses.lcaMass.rCM[2] +
      pChassis.frontAxle.masses.tieMass.m*pChassis.frontAxle.masses.tieMass.rCM[2] +
      pChassis.rearAxle.masses.unsprungMass.m*pChassis.rearAxle.masses.unsprungMass.rCM[2] +
      pChassis.rearAxle.masses.ucaMass.m*pChassis.rearAxle.masses.ucaMass.rCM[2] +
      pChassis.rearAxle.masses.lcaMass.m*pChassis.rearAxle.masses.lcaMass.rCM[2] +
      pChassis.rearAxle.masses.tieMass.m*pChassis.rearAxle.masses.tieMass.rCM[2]
    )/pInitializationMass,
    (
      pChassis.frame.mass.m*pChassis.frame.mass.rCM[3] +
      pChassis.frontAxle.masses.unsprungMass.m*pChassis.frontAxle.masses.unsprungMass.rCM[3] +
      pChassis.frontAxle.masses.ucaMass.m*pChassis.frontAxle.masses.ucaMass.rCM[3] +
      pChassis.frontAxle.masses.lcaMass.m*pChassis.frontAxle.masses.lcaMass.rCM[3] +
      pChassis.frontAxle.masses.tieMass.m*pChassis.frontAxle.masses.tieMass.rCM[3] +
      pChassis.rearAxle.masses.unsprungMass.m*pChassis.rearAxle.masses.unsprungMass.rCM[3] +
      pChassis.rearAxle.masses.ucaMass.m*pChassis.rearAxle.masses.ucaMass.rCM[3] +
      pChassis.rearAxle.masses.lcaMass.m*pChassis.rearAxle.masses.lcaMass.rCM[3] +
      pChassis.rearAxle.masses.tieMass.m*pChassis.rearAxle.masses.tieMass.rCM[3]
    )/pInitializationMass
  } "Default chassis start position when no QSS pose is supplied";

  extends BobLib.Chassis.BaseClasses.PartialChassis(
    chassisReferencePosition = pInitializationCG,
    final frontWheelRadius = pChassis.frontAxle.tire.wheel.geometry.R0,
    final rearWheelRadius = pChassis.rearAxle.tire.wheel.geometry.R0,
    final contactPatchPosition_1 =
      pChassis.frontAxle.geometry.wheelCenter +
      Frames.resolve1(
        Frames.axesRotations(
          {1, 2, 3},
          {
            Modelica.Units.Conversions.from_deg(pChassis.frontAxle.tire.wheel.geometry.staticGamma),
            0,
            Modelica.Units.Conversions.from_deg(pChassis.frontAxle.tire.wheel.geometry.staticAlpha)
          },
          {0, 0, 0}),
        {0, 0, -pChassis.frontAxle.tire.wheel.geometry.R0}),
    final contactPatchPosition_2 =
      Vector.mirrorXZ(
        pChassis.frontAxle.geometry.wheelCenter +
        Frames.resolve1(
          Frames.axesRotations(
            {1, 2, 3},
            {
              Modelica.Units.Conversions.from_deg(pChassis.frontAxle.tire.wheel.geometry.staticGamma),
              0,
              Modelica.Units.Conversions.from_deg(pChassis.frontAxle.tire.wheel.geometry.staticAlpha)
            },
            {0, 0, 0}),
          {0, 0, -pChassis.frontAxle.tire.wheel.geometry.R0})),
    final contactPatchPosition_3 =
      pChassis.rearAxle.geometry.wheelCenter +
      Frames.resolve1(
        Frames.axesRotations(
          {1, 2, 3},
          {
            Modelica.Units.Conversions.from_deg(pChassis.rearAxle.tire.wheel.geometry.staticGamma),
            0,
            Modelica.Units.Conversions.from_deg(pChassis.rearAxle.tire.wheel.geometry.staticAlpha)
          },
          {0, 0, 0}),
        {0, 0, -pChassis.rearAxle.tire.wheel.geometry.R0}),
    final contactPatchPosition_4 =
      Vector.mirrorXZ(
        pChassis.rearAxle.geometry.wheelCenter +
        Frames.resolve1(
          Frames.axesRotations(
            {1, 2, 3},
            {
              Modelica.Units.Conversions.from_deg(pChassis.rearAxle.tire.wheel.geometry.staticGamma),
              0,
              Modelica.Units.Conversions.from_deg(pChassis.rearAxle.tire.wheel.geometry.staticAlpha)
            },
            {0, 0, 0}),
          {0, 0, -pChassis.rearAxle.tire.wheel.geometry.R0})),
    final frontLeftRideHeightOffset =
      pChassis.frontAxle.rideHeightReference -
      {pChassis.frontAxle.geometry.wheelCenter[1], 0, pChassis.frontAxle.geometry.wheelCenter[3]},
    final frontRightRideHeightOffset =
      Vector.mirrorXZ(
        pChassis.frontAxle.rideHeightReference -
        {pChassis.frontAxle.geometry.wheelCenter[1], 0, pChassis.frontAxle.geometry.wheelCenter[3]}),
    final rearLeftRideHeightOffset =
      pChassis.rearAxle.rideHeightReference -
      {pChassis.rearAxle.geometry.wheelCenter[1], 0, pChassis.rearAxle.geometry.wheelCenter[3]},
    final rearRightRideHeightOffset =
      Vector.mirrorXZ(
        pChassis.rearAxle.rideHeightReference -
        {pChassis.rearAxle.geometry.wheelCenter[1], 0, pChassis.rearAxle.geometry.wheelCenter[3]}),
    redeclare replaceable FrAxleModel frAxleDW(pAxle = pChassis.frontAxle,
fixInitialLeftLowerArmAngle = fixInitialSuspensionAngles,
fixInitialRightLowerArmAngle = fixInitialSuspensionAngles,
fixInitialLowerArmAngularVelocities = fixInitialSuspensionAngles,
initialLeftLowerArmAngle = initialFrontLeftLowerArmAngle,
initialRightLowerArmAngle = initialFrontRightLowerArmAngle),
    redeclare replaceable RrAxleModel rrAxleDW(pAxle = pChassis.rearAxle,
steerable = false,
fixInitialLeftLowerArmAngle = fixInitialSuspensionAngles,
fixInitialRightLowerArmAngle = fixInitialSuspensionAngles,
fixInitialLowerArmAngularVelocities = fixInitialSuspensionAngles,
initialLeftLowerArmAngle = initialRearLeftLowerArmAngle,
initialRightLowerArmAngle = initialRearRightLowerArmAngle),
    redeclare replaceable FrameModel spaceFrame(pFrame = pChassis.frame));

initial equation

  // An initial equation keeps OMC from freezing the whole-record function result.
  pVehicleMass = BobLib.Utilities.Mechanics.Functions.combineSymmetricAxleMassRecords(
    pChassis.frame.mass, {pChassis.frontAxle.masses, pChassis.rearAxle.masses});
  frAxleDW.leftTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / frontWheelRadius;
  frAxleDW.rightTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / frontWheelRadius;
  rrAxleDW.leftTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / rearWheelRadius;
  rrAxleDW.rightTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / rearWheelRadius;

  annotation(Documentation(info = "<html>
<p>
Configurable BobLib double-wishbone chassis exposed as a VehicleInterfaces
two-axle chassis. This keeps the detailed BobLib suspension, tire, and frame
physics inside the VI chassis boundary while allowing experiment templates to
select front and rear axle variants. Tire and wheel defaults belong to the axle;
custom axle replacements retain their tire, wheel and compliance choices. Rear
steering is selected on rrAxleDW.steerable; the chassis defaults it to false.
</p>
</html>"));
end DoubleWishbone;
