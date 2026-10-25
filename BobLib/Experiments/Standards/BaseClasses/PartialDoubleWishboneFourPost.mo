within BobLib.Experiments.Standards.BaseClasses;

partial model PartialDoubleWishboneFourPost

  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Chassis.Suspension.Tires;
  import BobLib.Records.StandardRecord.FourPostEvalRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  import BobLib.Utilities.Math.Vector;

  // FourPostEval record

  // Bar record to override rate

  // Tire record

  replaceable record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord
    constrainedby BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord
    annotation(choicesAllMatching = true);
  replaceable model FrAxleModel =
    BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar
    constrainedby BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle
    annotation(choicesAllMatching = true, Dialog(tab = "Plant Models", group = "Front axle"));
  replaceable model RrAxleModel =
    BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar
    constrainedby BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle
    annotation(choicesAllMatching = true, Dialog(tab = "Plant Models", group = "Rear axle"));

  parameter VehicleRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));

  parameter StabarRecord pFrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0) "Front anti-roll-bar rig calibration; the default disables bar stiffness" annotation(Dialog(group = "Calibration"));

  parameter StabarRecord pRrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0) "Rear anti-roll-bar rig calibration; the default disables bar stiffness" annotation(Dialog(group = "Calibration"));

  extends PartialFourPostRig(
    redeclare replaceable FrAxleModel frAxleDW(
      final quasiStatic = true,
      pAxle(actuation = pVehicle.chassis.frontAxle.actuation,
        steering = pVehicle.chassis.frontAxle.steering,
        tire = pVehicle.chassis.frontAxle.tire,
        geometry = pVehicle.chassis.frontAxle.geometry,
        masses = pVehicle.chassis.frontAxle.masses),
      redeclare replaceable Tires.BaseTire leftTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
        redeclare replaceable Tires.Wheels.Wheel0DOF wheelModel,
        redeclare replaceable Tires.MF52.SlipModel.NoSlip slipModel),
      redeclare replaceable Tires.BaseTire rightTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
        redeclare replaceable Tires.Wheels.Wheel0DOF wheelModel,
        redeclare replaceable Tires.MF52.SlipModel.NoSlip slipModel)),
    redeclare replaceable RrAxleModel rrAxleDW(
      final quasiStatic = true,
      pAxle(actuation = pVehicle.chassis.rearAxle.actuation,
        steering = pVehicle.chassis.rearAxle.steering,
        tire = pVehicle.chassis.rearAxle.tire,
        geometry = pVehicle.chassis.rearAxle.geometry,
        masses = pVehicle.chassis.rearAxle.masses),
      redeclare replaceable Tires.BaseTire leftTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
        redeclare replaceable Tires.Wheels.Wheel0DOF wheelModel,
        redeclare replaceable Tires.MF52.SlipModel.NoSlip slipModel),
      redeclare replaceable Tires.BaseTire rightTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
        redeclare replaceable Tires.Wheels.Wheel0DOF wheelModel,
        redeclare replaceable Tires.MF52.SlipModel.NoSlip slipModel)),
    final frontAxleReference = frAxleDW.effectiveCenter,
    final rearAxleReference = rrAxleDW.effectiveCenter,
    final cpInitFL = pVehicle.chassis.frontAxle.geometry.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {Modelica.Units.Conversions.from_deg(pVehicle.chassis.frontAxle.tire.wheel.geometry.staticGamma), 0, Modelica.Units.Conversions.from_deg(pVehicle.chassis.frontAxle.tire.wheel.geometry.staticAlpha)},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.chassis.frontAxle.tire.wheel.geometry.R0}),
    final cpInitFR = Vector.mirrorXZ(cpInitFL),
    final cpInitRL = pVehicle.chassis.rearAxle.geometry.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {Modelica.Units.Conversions.from_deg(pVehicle.chassis.rearAxle.tire.wheel.geometry.staticGamma), 0, Modelica.Units.Conversions.from_deg(pVehicle.chassis.rearAxle.tire.wheel.geometry.staticAlpha)},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.chassis.rearAxle.tire.wheel.geometry.R0}),
    final cpInitRR = Vector.mirrorXZ(cpInitRL));

  // FourPostEval records
  FourPostEvalRecord frKnC;
  FourPostEvalRecord rrKnC;

protected

  // Front quantities
  Real frLeftDeltaVec[3];
  Real frLeftKingpinVec[3];
  Real frLeftGroundParam;
  Real frLeftGroundPoint[3];

  Real frRightDeltaVec[3];
  Real frRightKingpinVec[3];
  Real frRightGroundParam;
  Real frRightGroundPoint[3];

  // Rear quantities
  Real rrLeftDeltaVec[3];
  Real rrLeftKingpinVec[3];
  Real rrLeftGroundParam;
  Real rrLeftGroundPoint[3];

  Real rrRightDeltaVec[3];
  Real rrRightKingpinVec[3];
  Real rrRightGroundParam;
  Real rrRightGroundPoint[3];

equation

  // Front quantities
  frKnC.leftGamma = frAxleDW.leftTire.gamma;

  frLeftDeltaVec = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  frKnC.leftToe = atan(frLeftDeltaVec[2]/frLeftDeltaVec[1]);

  frLeftKingpinVec = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  frKnC.leftCaster = atan(-1*frLeftKingpinVec[1]/frLeftKingpinVec[3]);
  frKnC.leftKpi = atan(-1*frLeftKingpinVec[2]/frLeftKingpinVec[3]);

  frLeftGroundParam = (frAxleDW.leftCP.r_0[3] - frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/frLeftKingpinVec[3];
  frLeftGroundPoint = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + frLeftGroundParam*frLeftKingpinVec;
  frKnC.leftMechTrail = frLeftGroundPoint[1] - frAxleDW.leftCP.r_0[1];
  frKnC.leftMechScrub = frAxleDW.leftCP.r_0[2] - frLeftGroundPoint[2];

  frKnC.rightGamma = frAxleDW.rightTire.gamma;

  frRightDeltaVec = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  frKnC.rightToe = atan(frRightDeltaVec[2]/frRightDeltaVec[1]);

  frRightKingpinVec = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  frKnC.rightCaster = atan(-1*frRightKingpinVec[1]/frRightKingpinVec[3]);
  frKnC.rightKpi = atan(frRightKingpinVec[2]/frRightKingpinVec[3]);

  frRightGroundParam = (frAxleDW.rightCP.r_0[3] - frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/frRightKingpinVec[3];
  frRightGroundPoint = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + frRightGroundParam*frRightKingpinVec;
  frKnC.rightMechTrail = frRightGroundPoint[1] - frAxleDW.rightCP.r_0[1];
  frKnC.rightMechScrub = frRightGroundPoint[2] - frAxleDW.rightCP.r_0[2];

  frKnC.leftSpringLength = frAxleDW.leftSpringLength;
  frKnC.leftFz = frAxleDW.leftTire.Fz;
  frKnC.rightSpringLength = frAxleDW.rightSpringLength;
  frKnC.rightFz = frAxleDW.rightTire.Fz;
  frKnC.stabarAngle = frAxleDW.stabarAngle;

  frKnC.jackingForce = frChassisActuator.jackingOutput;
  frKnC.heave = frChassisActuator.heaveInput;
  frKnC.roll = frChassisActuator.rollInput;
  frKnC.fx = FL_ForceActuator.worldForce.force[1] + FR_ForceActuator.worldForce.force[1];
  frKnC.fy = FL_ForceActuator.worldForce.force[2] + FR_ForceActuator.worldForce.force[2];

  // Rear quantities
  rrKnC.leftGamma = rrAxleDW.leftTire.gamma;

  rrLeftDeltaVec = Frames.resolve1(rrAxleDW.leftCP.R, {1, 0, 0});
  rrKnC.leftToe = atan(rrLeftDeltaVec[2]/rrLeftDeltaVec[1]);

  rrLeftKingpinVec = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  rrKnC.leftCaster = atan(-1*rrLeftKingpinVec[1]/rrLeftKingpinVec[3]);
  rrKnC.leftKpi = atan(-1*rrLeftKingpinVec[2]/rrLeftKingpinVec[3]);

  rrLeftGroundParam = (rrAxleDW.leftCP.r_0[3] - rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/rrLeftKingpinVec[3];
  rrLeftGroundPoint = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + rrLeftGroundParam*rrLeftKingpinVec;
  rrKnC.leftMechTrail = rrLeftGroundPoint[1] - rrAxleDW.leftCP.r_0[1];
  rrKnC.leftMechScrub = rrAxleDW.leftCP.r_0[2] - rrLeftGroundPoint[2];

  rrKnC.rightGamma = rrAxleDW.rightTire.gamma;

  rrRightDeltaVec = Frames.resolve1(rrAxleDW.rightCP.R, {1, 0, 0});
  rrKnC.rightToe = atan(rrRightDeltaVec[2]/rrRightDeltaVec[1]);

  rrRightKingpinVec = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  rrKnC.rightCaster = atan(-1*rrRightKingpinVec[1]/rrRightKingpinVec[3]);
  rrKnC.rightKpi = atan(rrRightKingpinVec[2]/rrRightKingpinVec[3]);

  rrRightGroundParam = (rrAxleDW.rightCP.r_0[3] - rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/rrRightKingpinVec[3];
  rrRightGroundPoint = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + rrRightGroundParam*rrRightKingpinVec;
  rrKnC.rightMechTrail = rrRightGroundPoint[1] - rrAxleDW.rightCP.r_0[1];
  rrKnC.rightMechScrub = rrRightGroundPoint[2] - rrAxleDW.rightCP.r_0[2];

  rrKnC.leftSpringLength = rrAxleDW.leftSpringLength;
  rrKnC.leftFz = rrAxleDW.leftTire.Fz;
  rrKnC.rightSpringLength = rrAxleDW.rightSpringLength;
  rrKnC.rightFz = rrAxleDW.rightTire.Fz;
  rrKnC.stabarAngle = rrAxleDW.stabarAngle;

  rrKnC.jackingForce = rrChassisActuator.jackingOutput;
  rrKnC.heave = rrChassisActuator.heaveInput;
  rrKnC.roll = rrChassisActuator.rollInput;
  rrKnC.fx = RL_ForceActuator.worldForce.force[1] + RR_ForceActuator.worldForce.force[1];
  rrKnC.fy = RL_ForceActuator.worldForce.force[2] + RR_ForceActuator.worldForce.force[2];

end PartialDoubleWishboneFourPost;
