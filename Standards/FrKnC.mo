within BobLib.Standards;

model FrKnC
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoopRecord;
  
  inner parameter Boolean useInternalInput = true;
  
  parameter OrionRecord pVehicle;
  
//  parameter SIunits.Position upperFore_i[3] = pVehicle.pFrDW.upperFore_i "Upper control arm fore inboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position upperAft_i[3] = pVehicle.pFrDW.upperAft_i "Upper control arm aft inboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position lowerFore_i[3] = pVehicle.pFrDW.lowerFore_i "Lower control arm fore inboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position lowerAft_i[3] = pVehicle.pFrDW.lowerAft_i "Lower control arm aft inboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position upper_o[3] = pVehicle.pFrDW.upper_o "Upper control arm outboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position lower_o[3] = pVehicle.pFrDW.lower_o "Lower control arm outboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position tie_o[3] = pVehicle.pFrDW.tie_o "Tie rod outboard joint, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
//  parameter SIunits.Position wheelCenter[3] = pVehicle.pFrDW.wheelCenter "Centroid of volume enclosing wheel, expressed in chassis frame" annotation(
//    Evaluate = false,
//    Dialog(group = "Geometry"));
  
  parameter SIunits.Position upperFore_i[3] = {0.1016, 0.237744, 0.2143252};
  parameter SIunits.Position upperAft_i[3] = {-0.0680974, 0.2356358, 0.215138};
  parameter SIunits.Position lowerFore_i[3] = {0.1016, 0.226314, 0.08001};
  parameter SIunits.Position lowerAft_i[3] = {-0.0762, 0.226314, 0.08001};
  parameter SIunits.Position upper_o[3] = {-0.0092964, 0.5420106, 0.2679954};
  parameter SIunits.Position lower_o[3] = {0.0029972, 0.562991, 0.1139952};
  parameter SIunits.Position tie_o[3] = {0.0569976, 0.546989, 0.1522222};
  parameter SIunits.Position wheelCenter[3] = {0, 0.606110767456, 0.199898};
  
  Real _exposeUpperFore_i;
  Real _exposeUpperAft_i;
  Real _exposeLowerFore_i;
  Real _exposeLowerAft_i;
  Real _exposeUpper_o;
  Real _exposeLower_o;
  Real _exposeTie_o;
  Real _exposeWheelCenter;  
  
  output Real leftSpringLength;
  output Real rightSpringLength;
  output Real stabarAngle;
  
  extends BobLib.Standards.Templates.KnC(final leftCPFixed(r = leftCPInit),
                                         final rightCPFixed(r = rightCPInit));
  
  // Front axle
  BobLib.Vehicle.Chassis.Suspension.FrAxleDW frAxleDW(pAxle = pVehicle.pFrAxleDW,
                                                      pRack = pVehicle.pRack,
                                                      pStabar = pVehicle.pFrStabar,
                                                      pLeftPartialWheel = pVehicle.pFrPartialWheel,
                                                      pLeftDW = WishboneUprightLoopRecord(upperFore_i = upperFore_i,
                                                                                          upperAft_i = upperAft_i,
                                                                                          lowerFore_i = lowerFore_i,
                                                                                          lowerAft_i = lowerAft_i,
                                                                                          upper_o = upper_o,
                                                                                          lower_o = lower_o,
                                                                                          tie_o = tie_o,
                                                                                          wheelCenter = wheelCenter),
                                                      pLeftAxleMass = pVehicle.pFrAxleMass,
                                                      redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire leftTire(
                                                        redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel),
                                                      redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire rightTire(
                                                        redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel)) annotation(
    Placement(transformation(origin = {0, 50.4444}, extent = {{-34, -26.4444}, {34, 26.4444}})));

protected
  final parameter Real leftCPInit[3] = pVehicle.pFrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3}, {pVehicle.pFrPartialWheel.staticGamma*pi/180, 0, pVehicle.pFrPartialWheel.staticAlpha*pi/180}, {0, 0, 0}), {0, 0, -pVehicle.pFrPartialWheel.R0});
  final parameter Real rightCPInit[3] = Vector.mirrorXZ(leftCPInit);
  
  // Axle position
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toAxle(r = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]}, animation = false) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Steer input
  Modelica.Blocks.Sources.Ramp steerRamp(duration = 1, height = 0*Modelica.Constants.pi/180, startTime = 1) annotation(
    Placement(transformation(origin = {-60, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) annotation(
    Placement(transformation(origin = {-30, 90}, extent = {{-10, -10}, {10, 10}})));
  
equation
  
  _exposeUpperFore_i = upperFore_i[1] + upperFore_i[2] + upperFore_i[3];
  _exposeUpperAft_i = upperAft_i[1] + upperAft_i[2] + upperAft_i[3];
  _exposeLowerFore_i = lowerFore_i[1] + lowerFore_i[2] + lowerFore_i[3];
  _exposeLowerAft_i = lowerAft_i[1] + lowerAft_i[2] + lowerAft_i[3];
  _exposeUpper_o = upper_o[1] + upper_o[2] + upper_o[3];
  _exposeLower_o = lower_o[1] + lower_o[2] + lower_o[3];
  _exposeTie_o = tie_o[1] + tie_o[2] + tie_o[3];
  _exposeWheelCenter = wheelCenter[1] + wheelCenter[2] + wheelCenter[3];
  
  leftGamma = frAxleDW.leftTire.gamma;

  leftDeltaVec = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  leftToe = atan(leftDeltaVec[2]/leftDeltaVec[1]);

  leftKingpinVec = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  leftCaster = atan(-1*leftKingpinVec[1]/leftKingpinVec[3]);
  leftKpi = atan(-1*leftKingpinVec[2]/leftKingpinVec[3]);

  leftGroundParam = (frAxleDW.leftCP.r_0[3] - frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/leftKingpinVec[3];
  leftGroundPoint = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + leftGroundParam*leftKingpinVec;
  leftMechTrail = leftGroundPoint[1] - frAxleDW.leftCP.r_0[1];
  leftMechScrub = frAxleDW.leftCP.r_0[2] - leftGroundPoint[2];

  rightGamma = frAxleDW.rightTire.gamma;

  rightDeltaVec = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  rightToe = atan(rightDeltaVec[2]/rightDeltaVec[1]);

  rightKingpinVec = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  rightCaster = atan(-1*rightKingpinVec[1]/rightKingpinVec[3]);
  rightKpi = atan(rightKingpinVec[2]/rightKingpinVec[3]);

  rightGroundParam = (frAxleDW.rightCP.r_0[3] - frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/rightKingpinVec[3];
  rightGroundPoint = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + rightGroundParam*rightKingpinVec;
  rightMechTrail = rightGroundPoint[1] - frAxleDW.rightCP.r_0[1];
  rightMechScrub = rightGroundPoint[2] - frAxleDW.rightCP.r_0[2];

  leftSpringLength = frAxleDW.leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = frAxleDW.rightShockLinkage.lineForceWithMass.s;
  stabarAngle = frAxleDW.stabar.spring.phi_rel;

  connect(steerRamp.y, steerPosition.phi_ref) annotation(
    Line(points = {{-49, 90}, {-43, 90}}, color = {0, 0, 127}));
  connect(steerPosition.flange, frAxleDW.pinionFlange) annotation(
    Line(points = {{-20, 90}, {0, 90}, {0, 70}}));
  connect(toAxle.frame_b, frAxleDW.axleFrame) annotation(
    Line(points = {{0, 40}, {0, 60}}, color = {95, 95, 95}));
  connect(leftCPForce.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-60, 20}, {-47, 20}, {-47, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(rightCPForce.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{60, 20}, {47, 20}, {47, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(rollDOF.frame_b, toAxle.frame_a) annotation(
    Line(points = {{0, -30}, {0, 20}}, color = {95, 95, 95}));
  connect(left_DOF_xyz.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-40, 0}, {-40, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(right_DOF_xyz.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{40, 0}, {40, 50}, {34, 50}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.002));
end FrKnC;
