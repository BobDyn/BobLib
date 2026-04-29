within BobLib.Standards;

model FrKnCFMI
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  // FMI input
  input Real steerInput;
  
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
  
  // Model continues
  parameter OrionRecord pVehicle annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  extends BobLib.Standards.Templates.KinFMI(final toAxle(r = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]}),
                                            final leftCPFixed(r = leftCPInit),
                                            final rightCPFixed(r = rightCPInit));
  
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
  Modelica.Blocks.Sources.RealExpression steerSource(y = steerInput)  annotation(
    Placement(transformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}})));
  
protected
  // Calculated parameters
  final parameter Real leftCPInit[3] = pVehicle.pFrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                         {pVehicle.pFrPartialWheel.staticGamma*pi/180, 0, pVehicle.pFrPartialWheel.staticAlpha*pi/180},
                                                                                                         {0, 0, 0}),
                                                                                    {0, 0, -pVehicle.pFrPartialWheel.R0});
  final parameter Real rightCPInit[3] = Vector.mirrorXZ(leftCPInit);
  // Steer input
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}})));
  
equation
  leftShockLength = frAxleDW.leftShockLinkage.lineForceWithMass.s;
  rightShockLength = frAxleDW.rightShockLinkage.lineForceWithMass.s;
  
  // All visuals
  
  // Left base
  leftUpperFore_i = frAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  leftUpperAft_i = frAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  leftLowerFore_i = frAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  leftLowerAft_i = frAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  leftUpper_o = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  leftLower_o = frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  
  leftTie_i = frAxleDW.leftTieRod.frame_a.r_0;
  leftTie_o = frAxleDW.leftTieRod.frame_b.r_0;
  
  leftWheelCenter = frAxleDW.leftTire.chassisFrame.r_0;
  leftTire_ex = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  leftTire_ey = Frames.resolve1(frAxleDW.leftCP.R, {0, 1, 0});
  
  // Left enhanced
  leftBellcrankPivot = frAxleDW.leftBellcrank.mountFrame.r_0;
  leftBellcrankPickup1 = frAxleDW.leftBellcrank.pickupFrame1.r_0;
  leftBellcrankPickup2 = frAxleDW.leftBellcrank.pickupFrame2.r_0;
  leftBellcrankPickup3 = frAxleDW.leftBellcrank.pickupFrame3.r_0;
  leftRodMount = frAxleDW.leftPushrod.frame_b.r_0;
  leftShockMount = frAxleDW.leftShockLinkage.frame_b.r_0;
  
  leftBarEnd = frAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  leftArmEnd = frAxleDW.stabar.leftArmFrame.r_0;
  
  leftCP = frAxleDW.leftCP.r_0;
  leftCPForce = -1 * frAxleDW.leftCP.f;
  
  // Right base
  rightUpperFore_i = frAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  rightUpperAft_i = frAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  rightLowerFore_i = frAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  rightLowerAft_i = frAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  
  rightUpper_o = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  rightLower_o = frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  
  rightTie_i = frAxleDW.rightTieRod.frame_a.r_0;
  rightTie_o = frAxleDW.rightTieRod.frame_b.r_0;
  
  rightWheelCenter = frAxleDW.rightTire.chassisFrame.r_0;
  rightTire_ex = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  rightTire_ey = Frames.resolve1(frAxleDW.rightCP.R, {0, 1, 0});
  
  // Right enhanced
  rightBellcrankPivot = frAxleDW.rightBellcrank.mountFrame.r_0;
  rightBellcrankPickup1 = frAxleDW.rightBellcrank.pickupFrame1.r_0;
  rightBellcrankPickup2 = frAxleDW.rightBellcrank.pickupFrame2.r_0;
  rightBellcrankPickup3 = frAxleDW.rightBellcrank.pickupFrame3.r_0;
  rightRodMount = frAxleDW.rightPushrod.frame_b.r_0;
  rightShockMount = frAxleDW.rightShockLinkage.frame_b.r_0;
  
  rightBarEnd = frAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  rightArmEnd = frAxleDW.stabar.rightArmFrame.r_0;
  
  rightCP = frAxleDW.rightCP.r_0;
  rightCPForce = -1 * frAxleDW.rightCP.f;
  
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 70}}));
  connect(left_DOF_xyz.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-40, 0}, {-40, 50}, {-34, 50}}, color = {95, 95, 95}));
  connect(right_DOF_xyz.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{40, 0}, {40, 50}, {34, 50}}, color = {95, 95, 95}));
  connect(heaveDOF.frame_b, frAxleDW.axleFrame) annotation(
    Line(points = {{0, 30}, {0, 60}}, color = {95, 95, 95}));
  connect(steerSource.y, steerPosition.phi_ref) annotation(
    Line(points = {{-58, 110}, {-42, 110}}, color = {0, 0, 127}));
  
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end FrKnCFMI;
