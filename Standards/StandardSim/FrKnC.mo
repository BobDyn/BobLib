within BobLib.Standards.StandardSim;

model FrKnC
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  import BobLib.Resources.VisualRecord.Chassis.Suspension.AxleDW_BC_ARB_VisualRecord;
  
  parameter OrionRecord pVehicle annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  extends BobLib.Standards.Templates.KnC(final toAxle(r = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]}),
                                         final leftCPFixed(r = leftCPInit),
                                         final rightCPFixed(r = rightCPInit));
  
  AxleDW_BC_ARB_VisualRecord vis;
  
  // Front axle
  BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_ARB frAxleDW(pAxle = AxleDWRecord(bellcrankPivot = pVehicle.pFrAxleDW.bellcrankPivot,
                                                                                  bellcrankPivotAxis = pVehicle.pFrAxleDW.bellcrankPivotAxis,
                                                                                  bellcrankPickup1 = pVehicle.pFrAxleDW.bellcrankPickup1,
                                                                                  bellcrankPickup2 = pVehicle.pFrAxleDW.bellcrankPickup2,
                                                                                  bellcrankPickup3 = pVehicle.pFrAxleDW.bellcrankPickup3,
                                                                                  rodMount = pVehicle.pFrAxleDW.rodMount,
                                                                                  shockMount = pVehicle.pFrAxleDW.shockMount,
                                                                                  springTable = [0, 0; 1, 0],
                                                                                  springFreeLength = pVehicle.pFrAxleDW.springFreeLength,
                                                                                  damperTable = [0, 0; 1, 0]),
                                                             pRack = pVehicle.pFrRack,
                                                             pStabar = StabarRecord(leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
                                                                                    leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
                                                                                    barRate = 0),
                                                             pLeftPartialWheel = pVehicle.pFrPartialWheel,
                                                             pLeftDW = pVehicle.pFrDW,
                                                             pLeftAxleMass = pVehicle.pFrAxleMass,
                                                             redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire leftTire(
                                                               redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel),
                                                             redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire rightTire(
                                                               redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel)) annotation(
    Placement(transformation(origin = {0, 50.4444}, extent = {{-39.75, -17.6667}, {39.75, 17.6667}})));

protected
  // Calculated parameters
  final parameter Real leftCPInit[3] = pVehicle.pFrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                         {pVehicle.pFrPartialWheel.staticGamma*pi/180, 0, pVehicle.pFrPartialWheel.staticAlpha*pi/180},
                                                                                                         {0, 0, 0}),
                                                                                    {0, 0, -pVehicle.pFrPartialWheel.R0});
  final parameter Real rightCPInit[3] = Vector.mirrorXZ(leftCPInit);
  
  // Steer input
  Modelica.Blocks.Sources.Ramp steerRamp(duration = 1, height = 0*Modelica.Constants.pi/180, startTime = 1) annotation(
    Placement(transformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}})));
  
equation
  knc.leftGamma = frAxleDW.leftTire.gamma;

  leftDeltaVec = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  knc.leftToe = atan(leftDeltaVec[2]/leftDeltaVec[1]);

  leftKingpinVec = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  knc.leftCaster = atan(-1*leftKingpinVec[1]/leftKingpinVec[3]);
  knc.leftKpi = atan(-1*leftKingpinVec[2]/leftKingpinVec[3]);

  leftGroundParam = (frAxleDW.leftCP.r_0[3] - frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/leftKingpinVec[3];
  leftGroundPoint = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + leftGroundParam*leftKingpinVec;
  knc.leftMechTrail = leftGroundPoint[1] - frAxleDW.leftCP.r_0[1];
  knc.leftMechScrub = frAxleDW.leftCP.r_0[2] - leftGroundPoint[2];

  knc.rightGamma = frAxleDW.rightTire.gamma;

  rightDeltaVec = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  knc.rightToe = atan(rightDeltaVec[2]/rightDeltaVec[1]);

  rightKingpinVec = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  knc.rightCaster = atan(-1*rightKingpinVec[1]/rightKingpinVec[3]);
  knc.rightKpi = atan(rightKingpinVec[2]/rightKingpinVec[3]);

  rightGroundParam = (frAxleDW.rightCP.r_0[3] - frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/rightKingpinVec[3];
  rightGroundPoint = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + rightGroundParam*rightKingpinVec;
  knc.rightMechTrail = rightGroundPoint[1] - frAxleDW.rightCP.r_0[1];
  knc.rightMechScrub = rightGroundPoint[2] - frAxleDW.rightCP.r_0[2];

  knc.leftSpringLength = frAxleDW.leftShockLinkage.lineForceWithMass.s;
  knc.rightSpringLength = frAxleDW.rightShockLinkage.lineForceWithMass.s;
  knc.stabarAngle = frAxleDW.stabar.spring.phi_rel;
  
  // All visuals
  
  // Left base
  vis.leftUpperFore_i = frAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.leftUpperAft_i = frAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.leftLowerFore_i = frAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.leftLowerAft_i = frAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  vis.leftUpper_o = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  vis.leftLower_o = frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  
  vis.leftTie_i = frAxleDW.leftTieRod.frame_a.r_0;
  vis.leftTie_o = frAxleDW.leftTieRod.frame_b.r_0;
  
  vis.leftWheelCenter = frAxleDW.leftTire.chassisFrame.r_0;
  vis.leftTire_ex = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  vis.leftTire_ey = Frames.resolve1(frAxleDW.leftCP.R, {0, 1, 0});
  
  // Left enhanced
  vis.leftBellcrankPivot = frAxleDW.leftBellcrank.mountFrame.r_0;
  vis.leftBellcrankPickup1 = frAxleDW.leftBellcrank.pickupFrame1.r_0;
  vis.leftBellcrankPickup2 = frAxleDW.leftBellcrank.pickupFrame2.r_0;
  vis.leftBellcrankPickup3 = frAxleDW.leftBellcrank.pickupFrame3.r_0;
  vis.leftRodMount = frAxleDW.leftPushrod.frame_b.r_0;
  vis.leftShockMount = frAxleDW.leftShockLinkage.frame_b.r_0;
  
  vis.leftBarEnd = frAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  vis.leftArmEnd = frAxleDW.stabar.leftArmFrame.r_0;
  
  vis.leftCP = frAxleDW.leftCP.r_0;
  vis.leftCPForce = -1 * frAxleDW.leftCP.f;
  
  // Right base
  vis.rightUpperFore_i = frAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.rightUpperAft_i = frAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.rightLowerFore_i = frAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.rightLowerAft_i = frAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  vis.rightUpper_o = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  vis.rightLower_o = frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  
  vis.rightTie_i = frAxleDW.rightTieRod.frame_a.r_0;
  vis.rightTie_o = frAxleDW.rightTieRod.frame_b.r_0;
  
  vis.rightWheelCenter = frAxleDW.rightTire.chassisFrame.r_0;
  vis.rightTire_ex = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  vis.rightTire_ey = Frames.resolve1(frAxleDW.rightCP.R, {0, 1, 0});
  
  // Right enhanced
  vis.rightBellcrankPivot = frAxleDW.rightBellcrank.mountFrame.r_0;
  vis.rightBellcrankPickup1 = frAxleDW.rightBellcrank.pickupFrame1.r_0;
  vis.rightBellcrankPickup2 = frAxleDW.rightBellcrank.pickupFrame2.r_0;
  vis.rightBellcrankPickup3 = frAxleDW.rightBellcrank.pickupFrame3.r_0;
  vis.rightRodMount = frAxleDW.rightPushrod.frame_b.r_0;
  vis.rightShockMount = frAxleDW.rightShockLinkage.frame_b.r_0;
  
  vis.rightBarEnd = frAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  vis.rightArmEnd = frAxleDW.stabar.rightArmFrame.r_0;
  
  vis.rightCP = frAxleDW.rightCP.r_0;
  vis.rightCPForce = -1 * frAxleDW.rightCP.f;
  
  connect(steerRamp.y, steerPosition.phi_ref) annotation(
    Line(points = {{-59, 110}, {-43, 110}}, color = {0, 0, 127}));
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 70}}));
  connect(leftCPForce.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-60, 20}, {-47, 20}, {-47, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(rightCPForce.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{60, 20}, {47, 20}, {47, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(left_DOF_xyz.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-40, 0}, {-40, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(right_DOF_xyz.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{40, 0}, {40, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(heaveDOF.frame_b, frAxleDW.axleFrame) annotation(
    Line(points = {{0, 30}, {0, 60}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 91, Tolerance = 1e-06, Interval = 0.002));
end FrKnC;
