within BobLib.Standards.StandardSim;

model RrKnC
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;
  
  parameter DWBCStabar_DWBCStabarRecord pVehicle annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  extends BobLib.Standards.Templates.KnC(final toAxle(r = {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]}),
                                         final leftCPFixed(r = leftCPInit),
                                         final rightCPFixed(r = rightCPInit));
  
  import BobLib.Resources.VisualRecord.Chassis.Suspension.AxleDW_BC_ARB_VisualRecord;
  
  AxleDW_BC_ARB_VisualRecord vis;
  
  // Rear axle
  BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_ARB rrAxleDW(pAxle = AxleDWRecord(bellcrankPivot = pVehicle.pRrAxleDW.bellcrankPivot,
                                                                                  bellcrankPivotAxis = pVehicle.pRrAxleDW.bellcrankPivotAxis,
                                                                                  bellcrankPickup1 = pVehicle.pRrAxleDW.bellcrankPickup1,
                                                                                  bellcrankPickup2 = pVehicle.pRrAxleDW.bellcrankPickup2,
                                                                                  bellcrankPickup3 = pVehicle.pRrAxleDW.bellcrankPickup3,
                                                                                  rodMount = pVehicle.pRrAxleDW.rodMount,
                                                                                  shockMount = pVehicle.pRrAxleDW.shockMount,
                                                                                  springTable = [0, 0; 1, 0],
                                                                                  springFreeLength = pVehicle.pRrAxleDW.springFreeLength,
                                                                                  damperTable = [0, 0; 1, 0]),
                                                      pRack = pVehicle.pRrRack,
                                                      pStabar = StabarRecord(leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
                                                                             leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
                                                                             barRate = 0),
                                                      pLeftPartialWheel = pVehicle.pRrPartialWheel,
                                                      pLeftDW = pVehicle.pRrDW,
                                                      pLeftAxleMass = pVehicle.pRrAxleMass,
                                                      redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire leftTire(
                                                        redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel),
                                                      redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire rightTire(
                                                        redeclare BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel.NoSlip slipModel)) annotation(
    Placement(transformation(origin = {0, 50.4444}, extent = {{-38.5714, -15}, {38.5714, 15}})));
  
  Modelica.Mechanics.MultiBody.Parts.Mounting1D steerLock annotation(
    Placement(transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}})));

protected
  // Calculated parameters
  final parameter Real leftCPInit[3] = pVehicle.pRrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                         {pVehicle.pRrPartialWheel.staticGamma*pi/180, 0, pVehicle.pRrPartialWheel.staticAlpha*pi/180},
                                                                                                         {0, 0, 0}),
                                                                                    {0, 0, -pVehicle.pRrPartialWheel.R0});
  final parameter Real rightCPInit[3] = Vector.mirrorXZ(leftCPInit);
  
equation
  knc.leftGamma = rrAxleDW.leftTire.gamma;
  
  leftDeltaVec = Frames.resolve1(rrAxleDW.leftCP.R, {1, 0, 0});
  knc.leftToe = atan(leftDeltaVec[2]/leftDeltaVec[1]);
  
  leftKingpinVec = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  knc.leftCaster = atan(-1*leftKingpinVec[1]/leftKingpinVec[3]);
  knc.leftKpi = atan(-1*leftKingpinVec[2]/leftKingpinVec[3]);
  
  leftGroundParam = (rrAxleDW.leftCP.r_0[3] - rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/leftKingpinVec[3];
  leftGroundPoint = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + leftGroundParam*leftKingpinVec;
  knc.leftMechTrail = leftGroundPoint[1] - rrAxleDW.leftCP.r_0[1];
  knc.leftMechScrub = rrAxleDW.leftCP.r_0[2] - leftGroundPoint[2];
  
  knc.rightGamma = rrAxleDW.rightTire.gamma;
  
  rightDeltaVec = Frames.resolve1(rrAxleDW.rightCP.R, {1, 0, 0});
  knc.rightToe = atan(rightDeltaVec[2]/rightDeltaVec[1]);
  
  rightKingpinVec = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  knc.rightCaster = atan(-1*rightKingpinVec[1]/rightKingpinVec[3]);
  knc.rightKpi = atan(rightKingpinVec[2]/rightKingpinVec[3]);
  
  rightGroundParam = (rrAxleDW.rightCP.r_0[3] - rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/rightKingpinVec[3];
  rightGroundPoint = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + rightGroundParam*rightKingpinVec;
  knc.rightMechTrail = rightGroundPoint[1] - rrAxleDW.rightCP.r_0[1];
  knc.rightMechScrub = rightGroundPoint[2] - rrAxleDW.rightCP.r_0[2];
  
  knc.leftSpringLength = rrAxleDW.leftShockLinkage.lineForceWithMass.s;
  knc.rightSpringLength = rrAxleDW.rightShockLinkage.lineForceWithMass.s;
  
  knc.stabarAngle = rrAxleDW.stabar.spring.phi_rel;
  
  // Left base
  vis.leftUpperFore_i = rrAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.leftUpperAft_i = rrAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.leftLowerFore_i = rrAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.leftLowerAft_i = rrAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  vis.leftUpper_o = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  vis.leftLower_o = rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  
  vis.leftTie_i = rrAxleDW.leftTieRod.frame_a.r_0;
  vis.leftTie_o = rrAxleDW.leftTieRod.frame_b.r_0;
  
  vis.leftWheelCenter = rrAxleDW.leftTire.chassisFrame.r_0;
  vis.leftTire_ex = Frames.resolve1(rrAxleDW.leftCP.R, {1, 0, 0});
  vis.leftTire_ey = Frames.resolve1(rrAxleDW.leftCP.R, {0, 1, 0});
  
  // Left enhanced
  vis.leftBellcrankPivot = rrAxleDW.leftBellcrank.mountFrame.r_0;
  vis.leftBellcrankPickup1 = rrAxleDW.leftBellcrank.pickupFrame1.r_0;
  vis.leftBellcrankPickup2 = rrAxleDW.leftBellcrank.pickupFrame2.r_0;
  vis.leftBellcrankPickup3 = rrAxleDW.leftBellcrank.pickupFrame3.r_0;
  vis.leftRodMount = rrAxleDW.leftPullrod.frame_b.r_0;
  vis.leftShockMount = rrAxleDW.leftShockLinkage.frame_b.r_0;
  
  vis.leftBarEnd = rrAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  vis.leftArmEnd = rrAxleDW.stabar.leftArmFrame.r_0;
  
  vis.leftCP = rrAxleDW.leftCP.r_0;
  vis.leftCPForce = -1 * rrAxleDW.leftCP.f;
  
  // Right base
  vis.rightUpperFore_i = rrAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.rightUpperAft_i = rrAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.rightLowerFore_i = rrAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.rightLowerAft_i = rrAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  vis.rightUpper_o = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  vis.rightLower_o = rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  
  vis.rightTie_i = rrAxleDW.rightTieRod.frame_a.r_0;
  vis.rightTie_o = rrAxleDW.rightTieRod.frame_b.r_0;
  
  vis.rightWheelCenter = rrAxleDW.rightTire.chassisFrame.r_0;
  vis.rightTire_ex = Frames.resolve1(rrAxleDW.rightCP.R, {1, 0, 0});
  vis.rightTire_ey = Frames.resolve1(rrAxleDW.rightCP.R, {0, 1, 0});
  
  // Right enhanced
  vis.rightBellcrankPivot = rrAxleDW.rightBellcrank.mountFrame.r_0;
  vis.rightBellcrankPickup1 = rrAxleDW.rightBellcrank.pickupFrame1.r_0;
  vis.rightBellcrankPickup2 = rrAxleDW.rightBellcrank.pickupFrame2.r_0;
  vis.rightBellcrankPickup3 = rrAxleDW.rightBellcrank.pickupFrame3.r_0;
  vis.rightRodMount = rrAxleDW.rightPullrod.frame_b.r_0;
  vis.rightShockMount = rrAxleDW.rightShockLinkage.frame_b.r_0;
  
  vis.rightBarEnd = rrAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  vis.rightArmEnd = rrAxleDW.stabar.rightArmFrame.r_0;
  
  vis.rightCP = rrAxleDW.rightCP.r_0;
  vis.rightCPForce = -1 * rrAxleDW.rightCP.f;
  
  connect(leftCPForce.frame_b, rrAxleDW.leftCP) annotation(
    Line(points = {{-60, 20}, {-47, 20}, {-47, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(rightCPForce.frame_b, rrAxleDW.rightCP) annotation(
    Line(points = {{60, 20}, {47, 20}, {47, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(left_DOF_xyz.frame_b, rrAxleDW.leftCP) annotation(
    Line(points = {{-40, 0}, {-40, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(right_DOF_xyz.frame_b, rrAxleDW.rightCP) annotation(
    Line(points = {{40, 0}, {40, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(heaveDOF.frame_b, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, 30}, {0, 60}}, color = {95, 95, 95}));
  connect(steerLock.frame_a, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, 80}, {0, 60}}, color = {95, 95, 95}));
  connect(steerLock.flange_b, rrAxleDW.steerFlange) annotation(
    Line(points = {{10, 90}, {60, 90}, {60, 56}, {0, 56}}));
  annotation(
    experiment(StartTime = 0, StopTime = 91, Tolerance = 1e-06, Interval = 0.002));
end RrKnC;
