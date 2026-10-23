within BobLib.Chassis.BaseClasses;
partial model PartialChassis

  "BobLib detailed chassis exposed through the VehicleInterfaces chassis contract"

  import Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;

  extends VehicleInterfaces.Icons.Chassis;

  extends BobLib.Chassis.BaseClasses.PartialChassisAssembly(
    final useRoadSurface = true,
    final exposeWheelBearings = includeWheelBearings or usingMultiBodyDriveline or includeRotorEffects);

  extends BobLib.Chassis.Interfaces.Base(
    final includeChassisFrame = true,
    final includeSteeringWheel = true);

  parameter SI.Velocity initialLongitudinalVelocity = 0
    "Initial longitudinal speed used to initialize tire spin" annotation(Dialog(group = "Initialization"));
  parameter SI.Position chassisReferencePosition[3] = {0, 0, 0}
    "Initial world position of the chassis reference frame" annotation(Dialog(group = "Initialization"));
  parameter SI.Angle chassisReferenceAngles[3] = {0, 0, 0}
    "Initial chassis orientation using the FreeMotion rotation sequence" annotation(Dialog(group = "Initialization"));
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(choices(checkBox = true), Evaluate = true, Dialog(tab = "Animation"));

  output SI.Force Fz_1 "Front-left tire normal load";
  output SI.Force Fz_2 "Front-right tire normal load";
  output SI.Force Fz_3 "Rear-left tire normal load";
  output SI.Force Fz_4 "Rear-right tire normal load";
  output SI.Length frontLeftSpringLength "Front-left suspension spring length";
  output SI.Length frontRightSpringLength "Front-right suspension spring length";
  output SI.Length rearLeftSpringLength "Rear-left suspension spring length";
  output SI.Length rearRightSpringLength "Rear-right suspension spring length";
  output SI.Angle leftSteerAngle "Front-left road wheel steer angle";
  output SI.Angle rightSteerAngle "Front-right road wheel steer angle";
  output SI.Angle avgSteerAngle "Average front road wheel steer angle";

  Modelica.Blocks.Interfaces.RealOutput rideHeight_1(unit = "m") "Front-left aero ride height";

  Modelica.Blocks.Interfaces.RealOutput rideHeight_2(unit = "m") "Front-right aero ride height";

  Modelica.Blocks.Interfaces.RealOutput rideHeight_3(unit = "m") "Rear-left aero ride height";

  Modelica.Blocks.Interfaces.RealOutput rideHeight_4(unit = "m") "Rear-right aero ride height";

  Modelica.Blocks.Interfaces.RealOutput vehicleSpeed(unit = "m/s")
    "Chassis speed magnitude published for controller subscribers";
  output SI.Velocity bodyVelocity[3]
    "Chassis reference-frame velocity resolved in the chassis frame";
  output SI.AngularVelocity bodyAngularVelocity[3]
    "Chassis angular velocity resolved in the chassis frame";
  output SI.Acceleration bodyAcceleration[3]
    "Chassis reference-frame acceleration resolved in the chassis frame";

protected

  // Concrete signal taps join expandable bus members without introducing another source.
  Modelica.Blocks.Interfaces.RealInput axleLoads[4](each unit = "N") "Corner loads received from the axle buses" annotation(Placement(visible = false, transformation(origin = {0, 60}, extent = {{0, 0}, {0, 0}})));

  Modelica.Blocks.Interfaces.RealInput axleSpringLengths[4](each unit = "m") "Corner spring lengths received from the axle buses" annotation(Placement(visible = false, transformation(origin = {0, 60}, extent = {{0, 0}, {0, 0}})));

  Modelica.Blocks.Interfaces.RealOutput longitudinalVelocity(unit = "m/s") = bodyVelocity[1] "Signed forward velocity";

  Modelica.Blocks.Interfaces.RealOutput lateralAcceleration(unit = "m/s2") = bodyAcceleration[2] "Lateral body acceleration";

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

  BobLib.Chassis.Interfaces.ChassisBus chassisBus "Chassis measurements" annotation(
    Placement(transformation(origin = {0, 60}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = chassisReferencePosition,
    animation = false) annotation(
      Placement(transformation(origin = {-25, -40}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    angles_fixed = true,
    angles_start = chassisReferenceAngles,
    enforceStates = true,
    useQuaternions = false,
    w_rel_a_fixed = true,
    w_rel_a_start = {0, 0, 0},
    v_rel_a(start = {initialLongitudinalVelocity, 0, 0}, each fixed = true)) annotation(
      Placement(transformation(origin = {0, -70}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround ground_1(
    final flatRoadFastPath = flatRoadFastPath,
    r = contactPatchPosition_1) annotation(
    Placement(transformation(origin = {-35, -68}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround ground_2(
    final flatRoadFastPath = flatRoadFastPath,
    r = contactPatchPosition_2) annotation(
    Placement(transformation(origin = {-35, 52}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround ground_3(
    final flatRoadFastPath = flatRoadFastPath,
    r = contactPatchPosition_3) annotation(
    Placement(transformation(origin = {35, -68}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround ground_4(
    final flatRoadFastPath = flatRoadFastPath,
    r = contactPatchPosition_4) annotation(
    Placement(transformation(origin = {35, 52}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));

  Real leftWheelVector[3];
  Real rightWheelVector[3];
  SI.Position rideHeightPoint_1[3];
  SI.Position rideHeightPoint_2[3];
  SI.Position rideHeightPoint_3[3];
  SI.Position rideHeightPoint_4[3];

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadSurface rideHeightSurface[4](
    each final flatRoadFastPath = flatRoadFastPath,
    r = {rideHeightPoint_1, rideHeightPoint_2, rideHeightPoint_3, rideHeightPoint_4},
    s_start = {contactPatchPosition_1[1], contactPatchPosition_2[1], contactPatchPosition_3[1], contactPatchPosition_4[1]},
    w_start = {contactPatchPosition_1[2], contactPatchPosition_2[2], contactPatchPosition_3[2], contactPatchPosition_4[2]})
    "Normal clearance at each sprung aero reference point" annotation(
      Placement(transformation(origin = {110, 60}, extent = {{-10, -6}, {10, 6}})));
  SI.Velocity chassisVelocity[3];

equation
  connect(
    wheelHub_1.bearingFrame,
    frAxleDW.leftBearing) annotation(Line(visible = exposeWheelBearings, points = {{-90, -100}, {-74, -100}, {-74, -46}, {-63, -46}}, color = {95, 95, 95}));
  connect(
    wheelHub_2.bearingFrame,
    frAxleDW.rightBearing) annotation(Line(visible = exposeWheelBearings, points = {{-90, 100}, {-74, 100}, {-74, 26}, {-63, 26}}, color = {95, 95, 95}));
  connect(
    wheelHub_3.bearingFrame,
    rrAxleDW.leftBearing) annotation(Line(visible = exposeWheelBearings, points = {{90, -100}, {74, -100}, {74, -46}, {63, -46}}, color = {95, 95, 95}));
  connect(
    wheelHub_4.bearingFrame,
    rrAxleDW.rightBearing) annotation(Line(visible = exposeWheelBearings, points = {{90, 100}, {74, 100}, {74, 26}, {63, 26}}, color = {95, 95, 95}));
  Fz_1 = axleLoads[1];
  Fz_2 = axleLoads[2];
  Fz_3 = axleLoads[3];
  Fz_4 = axleLoads[4];
  frontLeftSpringLength = axleSpringLengths[1];
  frontRightSpringLength = axleSpringLengths[2];
  rearLeftSpringLength = axleSpringLengths[3];
  rearRightSpringLength = axleSpringLengths[4];

  leftWheelVector =
    Frames.resolve2(
      frAxleDW.axleFrame.R,
      Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0}));
  rightWheelVector =
    Frames.resolve2(
      frAxleDW.axleFrame.R,
      Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0}));
  leftSteerAngle = Modelica.Math.atan2(leftWheelVector[2], leftWheelVector[1]);
  rightSteerAngle = Modelica.Math.atan2(rightWheelVector[2], rightWheelVector[1]);
  avgSteerAngle = (leftSteerAngle + rightSteerAngle) / 2;

  rideHeightPoint_1 = frAxleDW.axleFrame.r_0 + Frames.resolve1(
    frAxleDW.axleFrame.R,
    frontLeftRideHeightOffset);
  rideHeightPoint_2 = frAxleDW.axleFrame.r_0 + Frames.resolve1(
    frAxleDW.axleFrame.R,
    frontRightRideHeightOffset);
  rideHeightPoint_3 = rrAxleDW.axleFrame.r_0 + Frames.resolve1(
    rrAxleDW.axleFrame.R,
    rearLeftRideHeightOffset);
  rideHeightPoint_4 = rrAxleDW.axleFrame.r_0 + Frames.resolve1(
    rrAxleDW.axleFrame.R,
    rearRightRideHeightOffset);
  rideHeight_1 = rideHeightSurface[1].gap;
  rideHeight_2 = rideHeightSurface[2].gap;
  rideHeight_3 = rideHeightSurface[3].gap;
  rideHeight_4 = rideHeightSurface[4].gap;
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
    Line(points = {{0, 100}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));

  // Signals from each axle share one drawn bus route to the co-located internal taps.
  connect(
    axleLoads[1],
    frAxleDW.axleBus.leftNormalLoad) annotation(Line(points = {{-41, 18}, {-25, 18}, {-25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(axleLoads[1], chassisBus.Fz_1) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleSpringLengths[1],
    frAxleDW.axleBus.leftSpringLength) annotation(Line(points = {{-41, 18}, {-25, 18}, {-25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(
    axleSpringLengths[1],
    chassisBus.frontLeftSpringLength) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleLoads[2],
    frAxleDW.axleBus.rightNormalLoad) annotation(Line(points = {{-41, 18}, {-25, 18}, {-25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(axleLoads[2], chassisBus.Fz_2) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleSpringLengths[2],
    frAxleDW.axleBus.rightSpringLength) annotation(Line(points = {{-41, 18}, {-25, 18}, {-25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(
    axleSpringLengths[2],
    chassisBus.frontRightSpringLength) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleLoads[3],
    rrAxleDW.axleBus.leftNormalLoad) annotation(Line(points = {{41, 18}, {25, 18}, {25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(axleLoads[3], chassisBus.Fz_3) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleSpringLengths[3],
    rrAxleDW.axleBus.leftSpringLength) annotation(Line(points = {{41, 18}, {25, 18}, {25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(
    axleSpringLengths[3],
    chassisBus.rearLeftSpringLength) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleLoads[4],
    rrAxleDW.axleBus.rightNormalLoad) annotation(Line(points = {{41, 18}, {25, 18}, {25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(axleLoads[4], chassisBus.Fz_4) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    axleSpringLengths[4],
    rrAxleDW.axleBus.rightSpringLength) annotation(Line(points = {{41, 18}, {25, 18}, {25, 60}, {0, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(
    axleSpringLengths[4],
    chassisBus.rearRightSpringLength) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(rideHeight_1, chassisBus.rideHeight_1) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(rideHeight_2, chassisBus.rideHeight_2) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(rideHeight_3, chassisBus.rideHeight_3) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(rideHeight_4, chassisBus.rideHeight_4) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(vehicleSpeed, chassisBus.vehicleSpeed) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    longitudinalVelocity,
    chassisBus.longitudinalVelocity) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(
    lateralAcceleration,
    chassisBus.bodyAcceleration_2) annotation(Line(visible = false, points = {{0, 60}, {0, 60}}));
  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{-17, -40}, {-12, -40}, {-12, -85}, {0, -85}, {0, -78}}, color = {95, 95, 95}));
  connect(chassisFrame, cgFreeMotion.frame_b) annotation(
    Line(points = {{0, -100}, {20, -100}, {20, -62}, {0, -62}}, color = {95, 95, 95}));
  connect(cgFreeMotion.frame_b, spaceFrame.cgFrame) annotation(
    Line(points = {{0, -62}, {0, -22}}, color = {95, 95, 95}));
  connect(steeringWheel, frAxleDW.steerFlange) annotation(
    Line(visible = frAxleDW.steerable, points = {{-160, 0}, {-100, 0}, {-100, -10}, {-65, -10}}));
  connect(wheelHub_1.flange, frAxleDW.leftTorque) annotation(
    Line(points = {{-90, -100}, {-55, -100}, {-55, -46}}));
  connect(ground_1.frame_b, frAxleDW.leftCP) annotation(
    Line(points = {{-35, -60}, {-35, -46}, {-45, -46}}, color = {95, 95, 95}));
  connect(wheelHub_2.flange, frAxleDW.rightTorque) annotation(
    Line(points = {{-90, 100}, {-55, 100}, {-55, 26}}));
  connect(ground_2.frame_b, frAxleDW.rightCP) annotation(
    Line(points = {{-35, 44}, {-35, 26}, {-45, 26}}, color = {95, 95, 95}));
  connect(wheelHub_3.flange, rrAxleDW.leftTorque) annotation(
    Line(points = {{90, -100}, {55, -100}, {55, -46}}));
  connect(ground_3.frame_b, rrAxleDW.leftCP) annotation(
    Line(points = {{35, -60}, {35, -46}, {45, -46}}, color = {95, 95, 95}));
  connect(wheelHub_4.flange, rrAxleDW.rightTorque) annotation(
    Line(points = {{90, 100}, {55, 100}, {55, 26}}));
  connect(ground_4.frame_b, rrAxleDW.rightCP) annotation(
    Line(points = {{35, 44}, {35, 26}, {45, 26}}, color = {95, 95, 95}));

  annotation(Diagram(coordinateSystem(extent = {{-160, -100}, {160, 100}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
VehicleInterfaces-facing adapter inheriting BobLib's axle and body assembly.
The public contract follows
<code>BobLib.Chassis.Interfaces.Base</code>: four wheel hubs,
a chassis frame, a steering wheel flange, and the inherited control bus.
BobLib contact-patch frames, axle frames, tire models, the chassis
<code>FreeMotion</code> state holder, and shared MultiBody contact mechanics
are wired internally so the chassis can be substituted through the standard
VehicleInterfaces chassis contract.
The chassis publishes its per-corner aero ride-height measurements on
<code>controlBus.chassisBus.rideHeight_1</code> through
<code>rideHeight_4</code>, its signed forward velocity on the standard
<code>controlBus.chassisBus.longitudinalVelocity</code>, its speed magnitude on
<code>controlBus.chassisBus.vehicleSpeed</code>, tire normal loads on
<code>Fz_1</code> through <code>Fz_4</code>, and lateral acceleration on
<code>bodyAcceleration_2</code> so downstream subsystems consume chassis-owned
measurements through the shared VehicleInterfaces bus namespace.
</p>
<p>Ride height is the signed normal clearance from each sprung reference
point to the selected road, not its world-coordinate altitude. General roads
use the same locally unique orthogonal projection as tire contact. The flat-road
fast path reduces this to world Z only for the supported Z=0 horizontal road.</p>
</html>"));
end PartialChassis;
