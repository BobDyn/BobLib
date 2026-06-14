within BobLibVehicleInterfaces.Chassis;
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

  output SI.Force Fz_1 "Front-left tire normal load";
  output SI.Force Fz_2 "Front-right tire normal load";
  output SI.Force Fz_3 "Rear-left tire normal load";
  output SI.Force Fz_4 "Rear-right tire normal load";
  output SI.Angle leftSteerAngle "Front-left road wheel steer angle";
  output SI.Angle rightSteerAngle "Front-right road wheel steer angle";
  output SI.Angle avgSteerAngle "Average front road wheel steer angle";
  output SI.Length frontRideHeight "Average front aero ride height";
  output SI.Length rearRideHeight "Average rear aero ride height";

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

  replaceable BobLibVehicleInterfaces.Chassis.Internal.DetailedChassisBase detailedChassis
    "BobLib suspension, tire, and body implementation" annotation(
      Placement(transformation(origin = {0, 0}, extent = {{-58, -58}, {58, 58}})));

  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    "VehicleInterfaces chassis signal bus tap" annotation(
      Placement(transformation(extent = {{-130, 30}, {-110, 50}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation chassisFrameToCG(
    r = {0, 0, 0},
    animation = false) annotation(
      Placement(transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}})));

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

  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_1 annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_2 annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_3 annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground_4 annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRideHeight_1(
    r = frontLeftRideHeightOffset,
    animation = false) annotation(
      Placement(transformation(origin = {-40, 70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRideHeight_2(
    r = frontRightRideHeightOffset,
    animation = false) annotation(
      Placement(transformation(origin = {40, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRideHeight_3(
    r = rearLeftRideHeightOffset,
    animation = false) annotation(
      Placement(transformation(origin = {-40, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRideHeight_4(
    r = rearRightRideHeightOffset,
    animation = false) annotation(
      Placement(transformation(origin = {40, -70}, extent = {{-10, -10}, {10, 10}})));

  Real leftWheelVector[3];
  Real rightWheelVector[3];

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

  frontRideHeight = (toRideHeight_1.frame_b.r_0[3] + toRideHeight_2.frame_b.r_0[3]) / 2;
  rearRideHeight = (toRideHeight_3.frame_b.r_0[3] + toRideHeight_4.frame_b.r_0[3]) / 2;

  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{-158, 60}, {-120, 60}, {-120, 40}}, color = {255, 204, 51}, thickness = 0.5));

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

  connect(toRideHeight_1.frame_a, detailedChassis.frAxleFrame) annotation(
    Line(points = {{-30, 70}, {-20, 70}, {-20, 34}, {0, 34}}, color = {95, 95, 95}));
  connect(toRideHeight_2.frame_a, detailedChassis.frAxleFrame) annotation(
    Line(points = {{30, 70}, {20, 70}, {20, 34}, {0, 34}}, color = {95, 95, 95}));
  connect(toRideHeight_3.frame_a, detailedChassis.rrAxleFrame) annotation(
    Line(points = {{-30, -70}, {-20, -70}, {-20, -35}, {0, -35}}, color = {95, 95, 95}));
  connect(toRideHeight_4.frame_a, detailedChassis.rrAxleFrame) annotation(
    Line(points = {{30, -70}, {20, -70}, {20, -35}, {0, -35}}, color = {95, 95, 95}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces-facing wrapper for BobLib's detailed chassis implementation.
The public contract follows
<code>VehicleInterfaces.Chassis.Interfaces.TwoAxleBase</code>: four wheel hubs,
a chassis frame, a steering wheel flange, and the inherited control bus.
BobLib contact-patch frames, axle frames, tire models, and shared MultiBody
contact mechanics are wired internally so the chassis can be substituted through
the standard VehicleInterfaces chassis contract.
</p>
</html>"));
end ChassisBase;
