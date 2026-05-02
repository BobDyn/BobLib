within BobLib.Standards;

model VehicleFMI
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  // Import vehicle records
  import BobLib.Resources.VehicleDefn.OrionRecord;
  // Imoport standard record
  import BobLib.Resources.StandardRecord.VehicleFMIRecord;
  // Import visual record
  import BobLib.Resources.VisualRecord.Chassis.ChassisVisualRecord;
  
  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter OrionRecord pVehicle;
  
  parameter SIunits.Velocity initialVel = 10 "Initial velocity";
  
  input SIunits.Angle steerCommand;
  input SIunits.Torque driveTorqueCommand;
  
  Real bodyVels[3];
  Real bodyAccels[3];
  Real bodyAngles[3];
  
  // Standard outputs
  output SIunits.Acceleration accX;
  output SIunits.Acceleration accY;
  output SIunits.Angle handwheelAngle;
  output SIunits.Torque handwheelTorque;
  output SIunits.Angle leftSteerAngle;
  output SIunits.Angle rightSteerAngle;
  output SIunits.Angle roll;
  output SIunits.Angle sideslip;
  output SIunits.Velocity velX;
  output SIunits.Velocity velY;
  output SIunits.AngularVelocity yawVel;
  
  // Visual record
  ChassisVisualRecord vis;

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));

  // Vehicle
  BobLib.Vehicle.VehicleDW_RWD_Lock vehicle(pVehicle = pVehicle)  annotation(
    Placement(transformation(origin = {0, 20}, extent = {{-45, -50}, {45, 50}})));
  // Angle sensor
  Modelica.Mechanics.MultiBody.Sensors.RelativeAngles sprungAngles annotation(
    Placement(transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression steerExpression(y = steerCommand)  annotation(
    Placement(transformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression driveTorqueExpression(y = driveTorqueCommand)  annotation(
    Placement(transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}})));

protected
  // Calculated parameters
  final parameter Real cpInitFL[3] = pVehicle.pFrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {pVehicle.pFrPartialWheel.staticGamma*pi/180, 0, pVehicle.pFrPartialWheel.staticAlpha*pi/180},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.pFrPartialWheel.R0});
  final parameter Real cpInitFR[3] = Vector.mirrorXZ(cpInitFL);
  final parameter Real cpInitRL[3] = pVehicle.pRrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {pVehicle.pRrPartialWheel.staticGamma*pi/180, 0, pVehicle.pRrPartialWheel.staticAlpha*pi/180},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.pRrPartialWheel.R0});
  final parameter Real cpInitRR[3] = Vector.mirrorXZ(cpInitRL);
  
  Real leftWheelVector[3];
  Real rightWheelVector[3];
  // Initial geometry
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFL(r = cpInitFL, animation = false) annotation(
    Placement(transformation(origin = {-130, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFR(r = cpInitFR, animation = false) annotation(
    Placement(transformation(origin = {130, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRL(r = cpInitRL, animation = false) annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRR(r = cpInitRR, animation = false) annotation(
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(r = pVehicle.pSprungMass.rCM, animation = false) annotation(
    Placement(transformation(origin = {130, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(animation = false, r_rel_a(start = {0, 0, 0}, each fixed = true), enforceStates = true, v_rel_a(start = {initialVel, 0, 0}, each fixed = true), useQuaternions = false) annotation(
    Placement(transformation(origin = {100, 90}, extent = {{10, -10}, {-10, 10}})));
  // Ground interface
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));
  // Front steer position
  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition(exact = false) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}})));
  
initial equation
  vehicle.chassis.frAxleDW.leftTire.wheelModel.hubAxis.w = initialVel / pVehicle.pFrPartialWheel.R0;
  vehicle.chassis.frAxleDW.rightTire.wheelModel.hubAxis.w = initialVel / pVehicle.pFrPartialWheel.R0;
  vehicle.chassis.rrAxleDW.leftTire.wheelModel.hubAxis.w = initialVel / pVehicle.pRrPartialWheel.R0;
  vehicle.chassis.rrAxleDW.rightTire.wheelModel.hubAxis.w = initialVel / pVehicle.pRrPartialWheel.R0;
  
equation
  // General quantities
  bodyVels = Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);
  bodyAccels = Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.a_rel_a);
  bodyAngles = Frames.resolve2(cgFreeMotion.frame_b.R, sprungAngles.angles);
  
  leftWheelVector = Frames.resolve1(vehicle.chassis.frAxleFrame.R, Frames.resolve2(vehicle.frameFL.R, {1, 0, 0}));
  rightWheelVector = Frames.resolve1(vehicle.chassis.frAxleFrame.R, Frames.resolve2(vehicle.frameFR.R, {1, 0, 0}));
  
  // Output record
  leftSteerAngle = -1*atan(leftWheelVector[2]/leftWheelVector[1]);
  rightSteerAngle = -1*atan(rightWheelVector[2]/rightWheelVector[1]);
  handwheelAngle = vehicle.steerFlange.phi;
  
  // Kinematics
  velX = bodyVels[1];
  velY = bodyVels[2];
  yawVel = vehicle.chassis.spaceFrame.sprungBody.w_a[3];
  sideslip = atan(velY/velX);
  
  // Accelerations
  accX = bodyAccels[1];
  accY = bodyAccels[2];
  
  // Vehicle response
  roll = bodyAngles[1];
  
  handwheelTorque = -1*vehicle.steerFlange.tau; // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  
  // All visual variables
  
  // Front left (FL) base
  vis.frontAxle.leftUpperFore_i = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.frontAxle.leftUpperAft_i = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.frontAxle.leftLowerFore_i = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.frontAxle.leftLowerAft_i = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  vis.frontAxle.leftUpper_o = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  vis.frontAxle.leftLower_o = vehicle.chassis.frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  vis.frontAxle.leftTie_i = vehicle.chassis.frAxleDW.leftTieRod.frame_a.r_0;
  vis.frontAxle.leftTie_o = vehicle.chassis.frAxleDW.leftTieRod.frame_b.r_0;
  vis.frontAxle.leftWheelCenter = vehicle.chassis.frAxleDW.leftTire.chassisFrame.r_0;
  vis.frontAxle.leftTire_ex = Frames.resolve1(vehicle.chassis.frAxleDW.leftCP.R, {1, 0, 0});
  vis.frontAxle.leftTire_ey = Frames.resolve1(vehicle.chassis.frAxleDW.leftCP.R, {0, 1, 0});
  
  // FL enhanced
  vis.frontAxle.leftBellcrankPivot = vehicle.chassis.frAxleDW.leftBellcrank.mountFrame.r_0;
  vis.frontAxle.leftBellcrankPickup1 = vehicle.chassis.frAxleDW.leftBellcrank.pickupFrame1.r_0;
  vis.frontAxle.leftBellcrankPickup2 = vehicle.chassis.frAxleDW.leftBellcrank.pickupFrame2.r_0;
  vis.frontAxle.leftBellcrankPickup3 = vehicle.chassis.frAxleDW.leftBellcrank.pickupFrame3.r_0;
  vis.frontAxle.leftRodMount = vehicle.chassis.frAxleDW.leftPushrod.frame_b.r_0;
  vis.frontAxle.leftShockMount = vehicle.chassis.frAxleDW.leftShockLinkage.frame_b.r_0;
  vis.frontAxle.leftBarEnd = vehicle.chassis.frAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  vis.frontAxle.leftArmEnd = vehicle.chassis.frAxleDW.stabar.leftArmFrame.r_0;
  vis.frontAxle.leftCP = vehicle.chassis.frAxleDW.leftCP.r_0;
  vis.frontAxle.leftCPForce = -1*vehicle.chassis.frAxleDW.leftCP.f;
  
  // Front right (FR) base
  vis.frontAxle.rightUpperFore_i = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.frontAxle.rightUpperAft_i = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.frontAxle.rightLowerFore_i = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.frontAxle.rightLowerAft_i = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  vis.frontAxle.rightUpper_o = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  vis.frontAxle.rightLower_o = vehicle.chassis.frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  vis.frontAxle.rightTie_i = vehicle.chassis.frAxleDW.rightTieRod.frame_a.r_0;
  vis.frontAxle.rightTie_o = vehicle.chassis.frAxleDW.rightTieRod.frame_b.r_0;
  vis.frontAxle.rightWheelCenter = vehicle.chassis.frAxleDW.rightTire.chassisFrame.r_0;
  vis.frontAxle.rightTire_ex = Frames.resolve1(vehicle.chassis.frAxleDW.rightCP.R, {1, 0, 0});
  vis.frontAxle.rightTire_ey = Frames.resolve1(vehicle.chassis.frAxleDW.rightCP.R, {0, 1, 0});
  
  // FR enhanced
  vis.frontAxle.rightBellcrankPivot = vehicle.chassis.frAxleDW.rightBellcrank.mountFrame.r_0;
  vis.frontAxle.rightBellcrankPickup1 = vehicle.chassis.frAxleDW.rightBellcrank.pickupFrame1.r_0;
  vis.frontAxle.rightBellcrankPickup2 = vehicle.chassis.frAxleDW.rightBellcrank.pickupFrame2.r_0;
  vis.frontAxle.rightBellcrankPickup3 = vehicle.chassis.frAxleDW.rightBellcrank.pickupFrame3.r_0;
  vis.frontAxle.rightRodMount = vehicle.chassis.frAxleDW.rightPushrod.frame_b.r_0;
  vis.frontAxle.rightShockMount = vehicle.chassis.frAxleDW.rightShockLinkage.frame_b.r_0;
  vis.frontAxle.rightBarEnd = vehicle.chassis.frAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  vis.frontAxle.rightArmEnd = vehicle.chassis.frAxleDW.stabar.rightArmFrame.r_0;
  vis.frontAxle.rightCP = vehicle.chassis.frAxleDW.rightCP.r_0;
  vis.frontAxle.rightCPForce = -1*vehicle.chassis.frAxleDW.rightCP.f;
  
  // Rear left (RL) base
  vis.rearAxle.leftUpperFore_i = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.rearAxle.leftUpperAft_i = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.rearAxle.leftLowerFore_i = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.rearAxle.leftLowerAft_i = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  vis.rearAxle.leftUpper_o = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0;
  vis.rearAxle.leftLower_o = vehicle.chassis.rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  vis.rearAxle.leftTie_i = vehicle.chassis.rrAxleDW.leftTieRod.frame_a.r_0;
  vis.rearAxle.leftTie_o = vehicle.chassis.rrAxleDW.leftTieRod.frame_b.r_0;
  vis.rearAxle.leftWheelCenter = vehicle.chassis.rrAxleDW.leftTire.chassisFrame.r_0;
  vis.rearAxle.leftTire_ex = Frames.resolve1(vehicle.chassis.rrAxleDW.leftCP.R, {1, 0, 0});
  vis.rearAxle.leftTire_ey = Frames.resolve1(vehicle.chassis.rrAxleDW.leftCP.R, {0, 1, 0});
  
  // RL enhanced
  vis.rearAxle.leftBellcrankPivot = vehicle.chassis.rrAxleDW.leftBellcrank.mountFrame.r_0;
  vis.rearAxle.leftBellcrankPickup1 = vehicle.chassis.rrAxleDW.leftBellcrank.pickupFrame1.r_0;
  vis.rearAxle.leftBellcrankPickup2 = vehicle.chassis.rrAxleDW.leftBellcrank.pickupFrame2.r_0;
  vis.rearAxle.leftBellcrankPickup3 = vehicle.chassis.rrAxleDW.leftBellcrank.pickupFrame3.r_0;
  vis.rearAxle.leftRodMount = vehicle.chassis.rrAxleDW.leftPullrod.frame_b.r_0;
  vis.rearAxle.leftShockMount = vehicle.chassis.rrAxleDW.leftShockLinkage.frame_b.r_0;
  vis.rearAxle.leftBarEnd = vehicle.chassis.rrAxleDW.stabar.toLeftBarEnd.frame_b.r_0;
  vis.rearAxle.leftArmEnd = vehicle.chassis.rrAxleDW.stabar.leftArmFrame.r_0;
  vis.rearAxle.leftCP = vehicle.chassis.rrAxleDW.leftCP.r_0;
  vis.rearAxle.leftCPForce = -1*vehicle.chassis.rrAxleDW.leftCP.f;
  
  // Rear right (RR) base
  vis.rearAxle.rightUpperFore_i = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.upperFrameToFore.frame_b.r_0;
  vis.rearAxle.rightUpperAft_i = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.upperFrameToAft.frame_b.r_0;
  vis.rearAxle.rightLowerFore_i = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.lowerFrameToFore.frame_b.r_0;
  vis.rearAxle.rightLowerAft_i = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.lowerFrameToAft.frame_b.r_0;
  vis.rearAxle.rightUpper_o = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0;
  vis.rearAxle.rightLower_o = vehicle.chassis.rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  vis.rearAxle.rightTie_i = vehicle.chassis.rrAxleDW.rightTieRod.frame_a.r_0;
  vis.rearAxle.rightTie_o = vehicle.chassis.rrAxleDW.rightTieRod.frame_b.r_0;
  vis.rearAxle.rightWheelCenter = vehicle.chassis.rrAxleDW.rightTire.chassisFrame.r_0;
  vis.rearAxle.rightTire_ex = Frames.resolve1(vehicle.chassis.rrAxleDW.rightCP.R, {1, 0, 0});
  vis.rearAxle.rightTire_ey = Frames.resolve1(vehicle.chassis.rrAxleDW.rightCP.R, {0, 1, 0});
  
  // RR enhanced
  vis.rearAxle.rightBellcrankPivot = vehicle.chassis.rrAxleDW.rightBellcrank.mountFrame.r_0;
  vis.rearAxle.rightBellcrankPickup1 = vehicle.chassis.rrAxleDW.rightBellcrank.pickupFrame1.r_0;
  vis.rearAxle.rightBellcrankPickup2 = vehicle.chassis.rrAxleDW.rightBellcrank.pickupFrame2.r_0;
  vis.rearAxle.rightBellcrankPickup3 = vehicle.chassis.rrAxleDW.rightBellcrank.pickupFrame3.r_0;
  vis.rearAxle.rightRodMount = vehicle.chassis.rrAxleDW.rightPullrod.frame_b.r_0;
  vis.rearAxle.rightShockMount = vehicle.chassis.rrAxleDW.rightShockLinkage.frame_b.r_0;
  vis.rearAxle.rightBarEnd = vehicle.chassis.rrAxleDW.stabar.toRightBarEnd.frame_b.r_0;
  vis.rearAxle.rightArmEnd = vehicle.chassis.rrAxleDW.stabar.rightArmFrame.r_0;
  vis.rearAxle.rightCP = vehicle.chassis.rrAxleDW.rightCP.r_0;
  vis.rearAxle.rightCPForce = -1*vehicle.chassis.rrAxleDW.rightCP.f;
  
  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{120, 90}, {110, 90}}, color = {95, 95, 95}));
  connect(fixedFL.frame_b, groundFL.frame_a) annotation(
    Line(points = {{-120, 10}, {-110, 10}}, color = {95, 95, 95}));
  connect(fixedFR.frame_b, groundFR.frame_a) annotation(
    Line(points = {{120, 10}, {110, 10}}, color = {95, 95, 95}));
  connect(fixedRL.frame_b, groundRL.frame_a) annotation(
    Line(points = {{-120, -50}, {-110, -50}}, color = {95, 95, 95}));
  connect(fixedRR.frame_b, groundRR.frame_a) annotation(
    Line(points = {{120, -50}, {110, -50}}, color = {95, 95, 95}));
  connect(vehicle.frameRL, groundRL.frame_b) annotation(
    Line(points = {{-44, -22}, {-100, -22}, {-100, -40}}, color = {95, 95, 95}));
  connect(groundRR.frame_b, vehicle.frameRR) annotation(
    Line(points = {{100, -40}, {100, -22}, {46, -22}}, color = {95, 95, 95}));
  connect(vehicle.frameFL, groundFL.frame_b) annotation(
    Line(points = {{-44, 38}, {-100, 38}, {-100, 20}}, color = {95, 95, 95}));
  connect(vehicle.frameFR, groundFR.frame_b) annotation(
    Line(points = {{46, 38}, {100, 38}, {100, 20}}, color = {95, 95, 95}));
  connect(frSteerPosition.flange, vehicle.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 62}}));
  connect(cgFreeMotion.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{90, 90}, {70, 90}, {70, 20}, {46, 20}}, color = {95, 95, 95}));
  connect(world.frame_b, sprungAngles.frame_a) annotation(
    Line(points = {{-120, -110}, {50, -110}, {50, -80}}, color = {95, 95, 95}));
  connect(sprungAngles.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{50, -60}, {50, 20}, {46, 20}}, color = {95, 95, 95}));
  connect(steerExpression.y, frSteerPosition.phi_ref) annotation(
    Line(points = {{-58, 110}, {-42, 110}}, color = {0, 0, 127}));
  connect(driveTorqueExpression.y, vehicle.uPTNTorque) annotation(
    Line(points = {{-18, -50}, {0, -50}, {0, -34}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    Icon(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
  experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "-d=initialization,bltdump --maxSizeLinearTearing=5000");
end VehicleFMI;
