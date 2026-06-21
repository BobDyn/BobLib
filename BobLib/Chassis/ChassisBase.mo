within BobLib.Chassis;
partial model ChassisBase

  "BobLib detailed chassis exposed through the VehicleInterfaces chassis contract"
  extends VehicleInterfaces.Icons.Chassis;

  extends VehicleInterfaces.Chassis.Interfaces.TwoAxleBase(
    final includeChassisFrame = true,
    final includeSteeringWheel = true);

  import SI = Modelica.Units.SI;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  parameter SI.Velocity initialLongitudinalVelocity = 0
    "Initial longitudinal speed used to initialize tire spin";
  parameter SI.Position chassisReferencePosition[3] = {0, 0, 0}
    "Initial world position of the chassis reference frame";

  output SI.Force Fz_1 "Front-left tire normal load";
  output SI.Force Fz_2 "Front-right tire normal load";
  output SI.Force Fz_3 "Rear-left tire normal load";
  output SI.Force Fz_4 "Rear-right tire normal load";
  output SI.Angle leftSteerAngle "Front-left road wheel steer angle";
  output SI.Angle rightSteerAngle "Front-right road wheel steer angle";
  output SI.Angle avgSteerAngle "Average front road wheel steer angle";
  output SI.Length rideHeight_1 "Front-left aero ride height";
  output SI.Length rideHeight_2 "Front-right aero ride height";
  output SI.Length rideHeight_3 "Rear-left aero ride height";
  output SI.Length rideHeight_4 "Rear-right aero ride height";
  output SI.Velocity vehicleSpeed
    "Chassis speed magnitude published for controller subscribers";
  output SI.Velocity bodyVelocity[3]
    "Chassis reference-frame velocity resolved in the chassis frame";
  output SI.AngularVelocity bodyAngularVelocity[3]
    "Chassis angular velocity resolved in the chassis frame";
  output SI.Acceleration bodyAcceleration[3]
    "Chassis reference-frame acceleration resolved in the chassis frame";

protected
  import Modelica.Mechanics.MultiBody.Frames;

  parameter SI.Length frontWheelRadius(min = 1e-6) = 1
    "Front loaded wheel radius used for initial tire spin";
  parameter SI.Length rearWheelRadius(min = 1e-6) = 1
    "Rear loaded wheel radius used for initial tire spin";

  parameter SI.Position contactPatchPosition_1[3] = {0, 0, 0}
    "Initial front-left contact patch position in world frame";
  parameter SI.Position contactPatchPosition_2[3] = {0, 0, 0}
    "Initial front-right contact patch position in world frame";
  parameter SI.Position contactPatchPosition_3[3] = {0, 0, 0}
    "Initial rear-left contact patch position in world frame";
  parameter SI.Position contactPatchPosition_4[3] = {0, 0, 0}
    "Initial rear-right contact patch position in world frame";

  parameter SI.Position frontLeftRideHeightOffset[3] = {0, 0, 0}
    "Offset from front axle frame to front-left aero ride-height reference";
  parameter SI.Position frontRightRideHeightOffset[3] = {0, 0, 0}
    "Offset from front axle frame to front-right aero ride-height reference";
  parameter SI.Position rearLeftRideHeightOffset[3] = {0, 0, 0}
    "Offset from rear axle frame to rear-left aero ride-height reference";
  parameter SI.Position rearRightRideHeightOffset[3] = {0, 0, 0}
    "Offset from rear axle frame to rear-right aero ride-height reference";

  replaceable BobLib.Chassis.Internal.DetailedChassisBase detailedChassis
    "BobLib suspension, tire, and body implementation" annotation(
      Placement(transformation(origin = {0, 0}, extent = {{-58, -58}, {58, 58}})));

  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    "VehicleInterfaces chassis signal bus tap" annotation(
      Placement(transformation(extent = {{-130, 30}, {-110, 50}})));

  Modelica.Blocks.Sources.RealExpression rideHeight_1BusSignal(
    y = rideHeight_1) "Front-left ride height published to chassisBus" annotation(
      Placement(transformation(origin = {-94, 16}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression rideHeight_2BusSignal(
    y = rideHeight_2) "Front-right ride height published to chassisBus" annotation(
      Placement(transformation(origin = {-94, 6}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression rideHeight_3BusSignal(
    y = rideHeight_3) "Rear-left ride height published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -4}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression rideHeight_4BusSignal(
    y = rideHeight_4) "Rear-right ride height published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -14}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression vehicleSpeedBusSignal(
    y = vehicleSpeed) "Vehicle speed published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -24}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression normalLoad_1BusSignal(
    y = Fz_1) "Front-left tire normal load published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -34}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression normalLoad_2BusSignal(
    y = Fz_2) "Front-right tire normal load published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -44}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression normalLoad_3BusSignal(
    y = Fz_3) "Rear-left tire normal load published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -54}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression normalLoad_4BusSignal(
    y = Fz_4) "Rear-right tire normal load published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -64}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression bodyAcceleration_2BusSignal(
    y = bodyAcceleration[2]) "Body lateral acceleration published to chassisBus" annotation(
      Placement(transformation(origin = {-94, -74}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation chassisFrameToCG(
    r = {0, 0, 0},
    animation = false) annotation(
      Placement(transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = chassisReferencePosition,
    animation = false) annotation(
      Placement(transformation(origin = {-110, -86}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    angles_fixed = true,
    angles_start = {0, 0, 0},
    enforceStates = true,
    useQuaternions = false,
    w_rel_a_fixed = true,
    w_rel_a_start = {0, 0, 0},
    v_rel_a(start = {initialLongitudinalVelocity, 0, 0}, each fixed = true)) annotation(
      Placement(transformation(origin = {-130, -70}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedContactPatch_1(
    r = contactPatchPosition_1,
    animation = false) annotation(
      Placement(transformation(origin = {-130, 10}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedContactPatch_2(
    r = contactPatchPosition_2,
    animation = false) annotation(
      Placement(transformation(origin = {130, 10}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedContactPatch_3(
    r = contactPatchPosition_3,
    animation = false) annotation(
      Placement(transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedContactPatch_4(
    r = contactPatchPosition_4,
    animation = false) annotation(
      Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_1 annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_2 annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_3 annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_4 annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));

  Real leftWheelVector[3];
  Real rightWheelVector[3];
  SI.Position rideHeightPoint_1[3];
  SI.Position rideHeightPoint_2[3];
  SI.Position rideHeightPoint_3[3];
  SI.Position rideHeightPoint_4[3];
  SI.Velocity chassisVelocity[3];

initial equation
  detailedChassis.frAxleDW.leftTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / frontWheelRadius;
  detailedChassis.frAxleDW.rightTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / frontWheelRadius;
  detailedChassis.rrAxleDW.leftTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / rearWheelRadius;
  detailedChassis.rrAxleDW.rightTire.wheelModel.hubAxis.w =
    initialLongitudinalVelocity / rearWheelRadius;

equation
  Fz_1 = detailedChassis.frAxleDW.leftTire.Fz;
  Fz_2 = detailedChassis.frAxleDW.rightTire.Fz;
  Fz_3 = detailedChassis.rrAxleDW.leftTire.Fz;
  Fz_4 = detailedChassis.rrAxleDW.rightTire.Fz;

  leftWheelVector =
    Frames.resolve1(
      detailedChassis.frAxleFrame.R,
      Frames.resolve2(detailedChassis.frameFL.R, {1, 0, 0}));
  rightWheelVector =
    Frames.resolve1(
      detailedChassis.frAxleFrame.R,
      Frames.resolve2(detailedChassis.frameFR.R, {1, 0, 0}));
  leftSteerAngle = -1*atan(leftWheelVector[2] / leftWheelVector[1]);
  rightSteerAngle = -1*atan(rightWheelVector[2] / rightWheelVector[1]);
  avgSteerAngle = (leftSteerAngle + rightSteerAngle) / 2;

  rideHeightPoint_1 = detailedChassis.frAxleFrame.r_0 + Frames.resolve1(
    detailedChassis.frAxleFrame.R,
    frontLeftRideHeightOffset);
  rideHeightPoint_2 = detailedChassis.frAxleFrame.r_0 + Frames.resolve1(
    detailedChassis.frAxleFrame.R,
    frontRightRideHeightOffset);
  rideHeightPoint_3 = detailedChassis.rrAxleFrame.r_0 + Frames.resolve1(
    detailedChassis.rrAxleFrame.R,
    rearLeftRideHeightOffset);
  rideHeightPoint_4 = detailedChassis.rrAxleFrame.r_0 + Frames.resolve1(
    detailedChassis.rrAxleFrame.R,
    rearRightRideHeightOffset);
  rideHeight_1 = rideHeightPoint_1[3];
  rideHeight_2 = rideHeightPoint_2[3];
  rideHeight_3 = rideHeightPoint_3[3];
  rideHeight_4 = rideHeightPoint_4[3];
  bodyVelocity =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);
  bodyAngularVelocity =
    Frames.angularVelocity2(cgFreeMotion.frame_b.R);
  bodyAcceleration =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.a_rel_a);
  chassisVelocity = bodyVelocity;
  vehicleSpeed =
    sqrt(
      chassisVelocity[1]^2 +
      chassisVelocity[2]^2 +
      chassisVelocity[3]^2);
  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{-158, 60}, {-120, 60}, {-120, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(rideHeight_1BusSignal.y, chassisBus.rideHeight_1) annotation(
    Line(points = {{-85.2, 16}, {-112, 16}, {-112, 40}, {-120, 40}}, color = {0, 0, 127}));
  connect(rideHeight_2BusSignal.y, chassisBus.rideHeight_2) annotation(
    Line(points = {{-85.2, 6}, {-114, 6}, {-114, 40}, {-120, 40}}, color = {0, 0, 127}));
  connect(rideHeight_3BusSignal.y, chassisBus.rideHeight_3) annotation(
    Line(points = {{-85.2, -4}, {-116, -4}, {-116, 40}, {-120, 40}}, color = {0, 0, 127}));
  connect(rideHeight_4BusSignal.y, chassisBus.rideHeight_4) annotation(
    Line(points = {{-85.2, -14}, {-118, -14}, {-118, 40}, {-120, 40}}, color = {0, 0, 127}));
  connect(vehicleSpeedBusSignal.y, chassisBus.vehicleSpeed) annotation(
    Line(points = {{-85.2, -24}, {-120, -24}, {-120, 40}}, color = {0, 0, 127}));
  connect(normalLoad_1BusSignal.y, chassisBus.Fz_1) annotation(
    Line(points = {{-85.2, -34}, {-120, -34}, {-120, 40}}, color = {0, 0, 127}));
  connect(normalLoad_2BusSignal.y, chassisBus.Fz_2) annotation(
    Line(points = {{-85.2, -44}, {-120, -44}, {-120, 40}}, color = {0, 0, 127}));
  connect(normalLoad_3BusSignal.y, chassisBus.Fz_3) annotation(
    Line(points = {{-85.2, -54}, {-120, -54}, {-120, 40}}, color = {0, 0, 127}));
  connect(normalLoad_4BusSignal.y, chassisBus.Fz_4) annotation(
    Line(points = {{-85.2, -64}, {-120, -64}, {-120, 40}}, color = {0, 0, 127}));
  connect(bodyAcceleration_2BusSignal.y, chassisBus.bodyAcceleration_2) annotation(
    Line(points = {{-85.2, -74}, {-120, -74}, {-120, 40}}, color = {0, 0, 127}));

  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{-100, -86}, {-120, -86}, {-120, -70}}, color = {95, 95, 95}));
  connect(cgFreeMotion.frame_b, chassisFrame) annotation(
    Line(points = {{-140, -70}, {-160, -70}}, color = {95, 95, 95}));

  connect(wheelHub_1.flange, detailedChassis.flangeFL) annotation(
    Line(points = {{-90, -100}, {-112, -100}, {-112, 35}, {-58, 35}}));
  connect(wheelHub_2.flange, detailedChassis.flangeFR) annotation(
    Line(points = {{-90, 100}, {-90, 112}, {58, 112}, {58, 35}}));
  connect(wheelHub_3.flange, detailedChassis.flangeRL) annotation(
    Line(points = {{90, -100}, {90, -112}, {-58, -112}, {-58, -35}}));
  connect(wheelHub_4.flange, detailedChassis.flangeRR) annotation(
    Line(points = {{90, 100}, {112, 100}, {112, -35}, {58, -35}}));

  connect(chassisFrame, chassisFrameToCG.frame_a) annotation(
    Line(points = {{-160, -70}, {-80, -70}, {-80, -30}, {-40, -30}}, color = {95, 95, 95}));
  connect(chassisFrameToCG.frame_b, detailedChassis.cgFrame) annotation(
    Line(points = {{-20, -30}, {40, -30}, {40, 0}, {58, 0}}, color = {95, 95, 95}));
  connect(steeringWheel, detailedChassis.frSteerFlange) annotation(
    Line(points = {{0, 100}, {0, 48}}));

  connect(fixedContactPatch_1.frame_b, ground_1.frame_a) annotation(
    Line(points = {{-120, 10}, {-110, 10}}, color = {95, 95, 95}));
  connect(fixedContactPatch_2.frame_b, ground_2.frame_a) annotation(
    Line(points = {{120, 10}, {110, 10}}, color = {95, 95, 95}));
  connect(fixedContactPatch_3.frame_b, ground_3.frame_a) annotation(
    Line(points = {{-120, -50}, {-110, -50}}, color = {95, 95, 95}));
  connect(fixedContactPatch_4.frame_b, ground_4.frame_a) annotation(
    Line(points = {{120, -50}, {110, -50}}, color = {95, 95, 95}));

  connect(ground_1.frame_b, detailedChassis.frameFL) annotation(
    Line(points = {{-90, 10}, {-72, 10}, {-72, 20}, {-58, 20}}, color = {95, 95, 95}));
  connect(ground_2.frame_b, detailedChassis.frameFR) annotation(
    Line(points = {{90, 10}, {72, 10}, {72, 20}, {58, 20}}, color = {95, 95, 95}));
  connect(ground_3.frame_b, detailedChassis.frameRL) annotation(
    Line(points = {{-90, -50}, {-72, -50}, {-72, -49}, {-58, -49}}, color = {95, 95, 95}));
  connect(ground_4.frame_b, detailedChassis.frameRR) annotation(
    Line(points = {{90, -50}, {72, -50}, {72, -49}, {58, -49}}, color = {95, 95, 95}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces-facing wrapper for BobLib's detailed chassis implementation.
The public contract follows
<code>VehicleInterfaces.Chassis.Interfaces.TwoAxleBase</code>: four wheel hubs,
a chassis frame, a steering wheel flange, and the inherited control bus.
BobLib contact-patch frames, axle frames, tire models, the chassis
<code>FreeMotion</code> state holder, and shared MultiBody contact mechanics
are wired internally so the chassis can be substituted through the standard
VehicleInterfaces chassis contract.
The chassis publishes its per-corner aero ride-height measurements on
<code>controlBus.chassisBus.rideHeight_1</code> through
<code>rideHeight_4</code>, its speed on
<code>controlBus.chassisBus.vehicleSpeed</code>, tire normal loads on
<code>Fz_1</code> through <code>Fz_4</code>, and lateral acceleration on
<code>bodyAcceleration_2</code> so downstream subsystems consume chassis-owned
measurements through the shared VehicleInterfaces bus namespace.
</p>
</html>"));
end ChassisBase;
