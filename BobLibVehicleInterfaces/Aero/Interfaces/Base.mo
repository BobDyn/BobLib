within BobLibVehicleInterfaces.Aero.Interfaces;

partial model Base
  "Basic interface definition for an aerodynamic subsystem"
  import SI = Modelica.Units.SI;

  parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));
  parameter SI.Position mountOffset[3] = {0, 0, 0}
    "Offset from sprung chassis frame to aero load frame";

  Modelica.Blocks.Interfaces.RealInput frontRideHeight(
    quantity = "Length",
    unit = "m") "Front ride height used by the aerodynamic model" annotation(
      Placement(transformation(origin = {-96, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90),
        iconTransformation(origin = {-96, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput rearRideHeight(
    quantity = "Length",
    unit = "m") "Rear ride height used by the aerodynamic model" annotation(
      Placement(transformation(origin = {-32, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90),
        iconTransformation(origin = {-32, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput relativeAirSpeed(
    quantity = "Velocity",
    unit = "m/s") "Vehicle speed relative to the local air" annotation(
      Placement(transformation(origin = {32, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90),
        iconTransformation(origin = {32, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput airDensity(
    quantity = "Density",
    unit = "kg/m3") "Local air density" annotation(
      Placement(transformation(origin = {96, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90),
        iconTransformation(origin = {96, 120}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));

  output SI.Force force[3]
    "Aerodynamic force resolved in aero load frame";
  output SI.Torque torque[3]
    "Aerodynamic torque resolved in aero load frame";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungChassisFrame
    "Sprung chassis frame that carries the aerodynamic loads" annotation(
      Placement(transformation(extent = {{-176, -16}, {-144, 16}})));

protected
  BobLibVehicleInterfaces.Aero.Mounts.RigidMount rigidMount(
    r = mountOffset,
    animation = false) annotation(
      Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque aeroLoads(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b,
    animation = not headless) annotation(
      Placement(transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  aeroLoads.force = force;
  aeroLoads.torque = torque;

  connect(sprungChassisFrame, rigidMount.sprungChassisFrame) annotation(
    Line(points = {{-160, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(rigidMount.aeroFrame, aeroLoads.frame_b) annotation(
    Line(points = {{-40, 0}, {0, 0}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}}), graphics = {
      Rectangle(extent = {{-140, 60}, {140, -60}}, lineColor = {95, 95, 95}, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, radius = 8),
      Line(points = {{-120, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 1),
      Polygon(points = {{-25, 0}, {30, 30}, {95, 30}, {70, 0}, {95, -30}, {30, -30}, {-25, 0}}, lineColor = {0, 85, 170}, fillColor = {210, 230, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-80, 90}, {80, 60}}, textString = "Aero")}),
    Documentation(info = "<html>
<p>
This partial model defines the basic interfaces required for an aerodynamic
subsystem. Implementations provide <code>force</code> and <code>torque</code>
from relative airspeed, local air density, and chassis-derived ride-height
signals. The base interface applies those loads to
<code>sprungChassisFrame</code> through a rigid aero mount.
</p>
</html>"));
end Base;
