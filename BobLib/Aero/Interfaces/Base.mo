within BobLib.Aero.Interfaces;

partial model Base

  "Basic interface definition for an aerodynamic subsystem"

  import Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;

  parameter SI.Position mountOffset[3] = {0, 0, 0}
    "Offset from sprung chassis frame to aero load frame" annotation(Dialog(group = "Operating settings"));
  parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(choices(checkBox = true), Evaluate = true, Dialog(tab = "Animation"));

  BobLib.Utilities.Interfaces.ControlBus controlBus
    "VehicleInterfaces control bus carrying chassis measurements" annotation(
      Placement(transformation(origin = {-160, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}})));
  outer VehicleInterfaces.Atmospheres.Interfaces.Base atmosphere
    "Local atmosphere evaluated at the aero load point";

  output SI.Force force[3]
    "Aerodynamic force resolved in aero load frame";
  output SI.Torque torque[3]
    "Aerodynamic torque resolved in aero load frame";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungChassisFrame
    "Sprung chassis frame that carries the aerodynamic loads" annotation(
      Placement(
        transformation(origin = {0, -40}, extent = {{-176, -16}, {-144, 16}}),
        iconTransformation(extent = {{-176, -16}, {-144, 16}})));

protected

  BobLib.Chassis.Interfaces.ChassisBus chassisBus
    "VehicleInterfaces chassis signal bus tap" annotation(
      Placement(
        transformation(origin = {-10, 28}, extent = {{-120, 42}, {-100, 62}}),
        iconTransformation(extent = {{-120, 42}, {-100, 62}})));
  SI.Length rideHeight_1
    "Front-left ride height used by the aerodynamic model";
  SI.Length rideHeight_2
    "Front-right ride height used by the aerodynamic model";
  SI.Length rideHeight_3
    "Rear-left ride height used by the aerodynamic model";
  SI.Length rideHeight_4
    "Rear-right ride height used by the aerodynamic model";
  SI.Density airDensity
    "Local air density from the VI atmosphere function";
  SI.Velocity windVelocityWorld[3]
    "Wind velocity resolved in world frame from the VI atmosphere function";
  SI.Velocity bodyVelocity[3]
    "Sprung chassis velocity resolved in chassis frame";
  SI.Velocity windVelocityBody[3]
    "Wind velocity resolved in chassis frame";
  SI.Velocity relativeAirVelocity[3]
    "Vehicle velocity relative to local air, resolved in chassis frame";
  SI.Velocity relativeAirSpeed
    "Vehicle speed relative to local air";

  Modelica.Blocks.Interfaces.RealInput rideHeight_1BusTap(
    quantity = "Length",
    unit = "m") "Front-left ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {-60, 80}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 30}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_2BusTap(
    quantity = "Length",
    unit = "m") "Front-right ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {-60, 55}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 20}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_3BusTap(
    quantity = "Length",
    unit = "m") "Rear-left ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {-60, 30}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 10}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_4BusTap(
    quantity = "Length",
    unit = "m") "Rear-right ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {-60, 5}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 0}, extent = {{-8, -8}, {8, 8}})));

  BobLib.Aero.Mounts.RigidMount rigidMount(
    r = mountOffset,
    animation = false) annotation(
      Placement(transformation(origin = {-80, -40}, extent = {{-10, -6.666666666666667}, {10, 6.666666666666667}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque aeroLoads(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b,
    animation = not headless) annotation(
      Placement(transformation(origin = {40, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

equation
  aeroLoads.force = force;
  aeroLoads.torque = torque;
  rideHeight_1 = rideHeight_1BusTap;
  rideHeight_2 = rideHeight_2BusTap;
  rideHeight_3 = rideHeight_3BusTap;
  rideHeight_4 = rideHeight_4BusTap;
  airDensity = atmosphere.density(aeroLoads.frame_b.r_0);
  windVelocityWorld = atmosphere.windVelocity(aeroLoads.frame_b.r_0);
  bodyVelocity =
    Frames.resolve2(aeroLoads.frame_b.R, der(aeroLoads.frame_b.r_0));
  windVelocityBody =
    Frames.resolve2(aeroLoads.frame_b.R, windVelocityWorld);
  relativeAirVelocity =
    bodyVelocity - windVelocityBody;
  relativeAirSpeed =
    sqrt(
      relativeAirVelocity[1]^2 +
      relativeAirVelocity[2]^2 +
      relativeAirVelocity[3]^2);

  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{-160, 80}, {-120, 80}}, color = {255, 204, 51}, thickness = 0.5));

  connect(sprungChassisFrame, rigidMount.sprungChassisFrame) annotation(
    Line(points = {{-160, -40}, {-88.3333, -40}}, color = {95, 95, 95}));
  connect(rigidMount.aeroFrame, aeroLoads.frame_b) annotation(
    Line(points = {{-71.6667, -40}, {30, -40}}, color = {95, 95, 95}));
  connect(chassisBus.rideHeight_1, rideHeight_1BusTap) annotation(
    Line(points = {{-120, 80}, {-60, 80}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_2, rideHeight_2BusTap) annotation(
    Line(points = {{-120, 80}, {-86, 80}, {-86, 55}, {-60, 55}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_3, rideHeight_3BusTap) annotation(
    Line(points = {{-120, 80}, {-94, 80}, {-94, 30}, {-60, 30}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_4, rideHeight_4BusTap) annotation(
    Line(points = {{-120, 80}, {-102, 80}, {-102, 5}, {-60, 5}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-160, -60}, {100, 100}})),
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-160, -100}, {160, 100}}), graphics = {Text(extent = {{-100, 130}, {100, 105}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0),
      Rectangle(extent = {{-140, 60}, {140, -60}}, lineColor = {95, 95, 95}, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, radius = 8),
      Line(points = {{-120, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 1),
      Polygon(points = {{-25, 0}, {30, 30}, {95, 30}, {70, 0}, {95, -30}, {30, -30}, {-25, 0}}, lineColor = {0, 85, 170}, fillColor = {210, 230, 255}, fillPattern = FillPattern.Solid)}),
    Documentation(info = "<html>
<p>
This partial model defines the basic interfaces required for an aerodynamic
subsystem. Implementations provide <code>force</code> and <code>torque</code>
from chassis-owned ride heights, atmosphere-owned density and wind, and the
aero load-point velocity (including mount-offset rotational motion). Ride heights are read from
<code>controlBus.chassisBus</code>; density and wind are evaluated using the shared VI <code>outer atmosphere</code> functions at the aero load point. The base interface applies those loads to
<code>sprungChassisFrame</code> through a rigid aero mount.
</p>
</html>"));
end Base;