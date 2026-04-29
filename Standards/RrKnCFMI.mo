within BobLib.Standards;

model RrKnCFMI
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  // FMI output
  // Wishbone (inner)
  output Real leftUpperFore_i[3];
  output Real leftUpperAft_i[3];
  output Real leftLowerFore_i[3];
  output Real leftLowerAft_i[3];

  // Upright (outer)
  output Real leftUpper_o[3];
  output Real leftLower_o[3];

  // Steering
  output Real leftTie_i[3];
  output Real leftTie_o[3];

  // Wheel
  output Real leftWheelCenter[3];
  output Real leftTire_ex[3];
  output Real leftTire_ey[3];
  
  // Contact patch and loads
  output Real leftCP[3];
  output Real leftCPForce[3];

  // Left axle
  output Real leftBellcrankPivot[3];
  output Real leftBellcrankPickup1[3];
  output Real leftBellcrankPickup2[3];
  output Real leftBellcrankPickup3[3];
  output Real leftRodMount[3];
  output Real leftShockMount[3];
  
  output Real leftShockLength;
  
  output Real leftBarEnd[3];
  output Real leftArmEnd[3];
  
  // Right vals

  // Wishbone (inner)
  output Real rightUpperFore_i[3];
  output Real rightUpperAft_i[3];
  output Real rightLowerFore_i[3];
  output Real rightLowerAft_i[3];

  // Upright (outer)
  output Real rightUpper_o[3];
  output Real rightLower_o[3];

  // Steering
  output Real rightTie_i[3];
  output Real rightTie_o[3];

  // Wheel
  output Real rightWheelCenter[3];
  output Real rightTire_ex[3];
  output Real rightTire_ey[3];
  
  // Contact patch and loads
  output Real rightCP[3];
  output Real rightCPForce[3];
  
  // Right axle
  output Real rightBellcrankPivot[3];
  output Real rightBellcrankPickup1[3];
  output Real rightBellcrankPickup2[3];
  output Real rightBellcrankPickup3[3];
  output Real rightRodMount[3];
  output Real rightShockMount[3];
  
  output Real rightShockLength;
  
  output Real rightBarEnd[3];
  output Real rightArmEnd[3];
  
  parameter OrionRecord pVehicle annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  extends BobLib.Standards.Templates.KinFMI(final toAxle(r = {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]}),
                                            final leftCPFixed(r = leftCPInit),
                                            final rightCPFixed(r = rightCPInit));
  
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
  leftShockLength = rrAxleDW.leftShockLinkage.lineForceWithMass.s;
  rightShockLength = rrAxleDW.rightShockLinkage.lineForceWithMass.s;
  
  // All visuals
  
  // Left base
  leftUpperFore_i = rrAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  leftUpperAft_i = rrAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  leftLowerFore_i = rrAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  leftLowerAft_i = rrAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  leftUpper_o = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  leftLower_o = rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  
  leftTie_i = rrAxleDW.leftTieRod.frame_a.r_0;
  leftTie_o = rrAxleDW.leftTieRod.frame_b.r_0;
  
  leftWheelCenter = rrAxleDW.leftTire.chassisFrame.r_0;
  leftTire_ex = Frames.resolve1(rrAxleDW.leftCP.R, {1, 0, 0});
  leftTire_ey = Frames.resolve1(rrAxleDW.leftCP.R, {0, 1, 0});
  
  // Left enhanced
  leftBellcrankPivot = rrAxleDW.leftBellcrank.mountFrame.r_0;
  leftBellcrankPickup1 = rrAxleDW.leftBellcrank.pickupFrame1.r_0;
  leftBellcrankPickup2 = rrAxleDW.leftBellcrank.pickupFrame2.r_0;
  leftBellcrankPickup3 = rrAxleDW.leftBellcrank.pickupFrame3.r_0;
  leftRodMount = rrAxleDW.leftPullrod.frame_b.r_0;
  leftShockMount = rrAxleDW.leftShockLinkage.frame_b.r_0;
  
  leftBarEnd = rrAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  leftArmEnd = rrAxleDW.stabar.leftArmFrame.r_0;
  
  leftCP = rrAxleDW.leftCP.r_0;
  leftCPForce = -1 * rrAxleDW.leftCP.f;
  
  // Right base
  rightUpperFore_i = rrAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  rightUpperAft_i = rrAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  rightLowerFore_i = rrAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  rightLowerAft_i = rrAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  rightUpper_o = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  rightLower_o = rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  
  rightTie_i = rrAxleDW.rightTieRod.frame_a.r_0;
  rightTie_o = rrAxleDW.rightTieRod.frame_b.r_0;
  
  rightWheelCenter = rrAxleDW.rightTire.chassisFrame.r_0;
  rightTire_ex = Frames.resolve1(rrAxleDW.rightCP.R, {1, 0, 0});
  rightTire_ey = Frames.resolve1(rrAxleDW.rightCP.R, {0, 1, 0});
  
  // Right enhanced
  rightBellcrankPivot = rrAxleDW.rightBellcrank.mountFrame.r_0;
  rightBellcrankPickup1 = rrAxleDW.rightBellcrank.pickupFrame1.r_0;
  rightBellcrankPickup2 = rrAxleDW.rightBellcrank.pickupFrame2.r_0;
  rightBellcrankPickup3 = rrAxleDW.rightBellcrank.pickupFrame3.r_0;
  rightRodMount = rrAxleDW.rightPullrod.frame_b.r_0;
  rightShockMount = rrAxleDW.rightShockLinkage.frame_b.r_0;
  
  rightBarEnd = rrAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  rightArmEnd = rrAxleDW.stabar.rightArmFrame.r_0;
  
  rightCP = rrAxleDW.rightCP.r_0;
  rightCPForce = -1 * rrAxleDW.rightCP.f;
  
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
end RrKnCFMI;
