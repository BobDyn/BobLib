within BobLib.Chassis.Suspension.Interfaces;

partial model AxleBase "Mechanical boundary shared by suspension axle architectures"

  output Modelica.Units.SI.Torque leftContactTorque = 0 "Contact force contribution to left wheel spin torque";
  output Modelica.Units.SI.Torque rightContactTorque = 0 "Contact force contribution to right wheel spin torque";
  parameter Boolean useRoadSurface = false "Use VI surface normal and friction in tire physics" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean includeRotorEffects = false
    "Include rotor gyroscopic and carrier-acceleration effects" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean steerable = true "Expose the steering input; otherwise lock the rack" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Boolean flatRoadFastPath = false "Use the horizontal z=0 road specialization; requires a compatible VI road" annotation(Dialog(group = "Advanced"), choices(checkBox = true), Evaluate = true);

  BobLib.Chassis.Suspension.Interfaces.AxleBus axleBus "Axle-owned measurements" annotation(
    Placement(transformation(origin = {180, 140}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {140, -20}, extent = {{-10, -10}, {10, 10}})));

  parameter Boolean exposeWheelBearings = true "Expose upright support frames for caliper and halfshaft reactions" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b leftBearing if exposeWheelBearings "Left upright support at the wheel axis; non-spinning caliper attachment" annotation(
    Placement(transformation(origin = {-180, 110}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-180, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rightBearing if exposeWheelBearings "Right upright support at the wheel axis; non-spinning caliper attachment" annotation(
    Placement(transformation(origin = {180, 110}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {180, 90}, extent = {{-10, -10}, {10, 10}})));

  // Interface frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a axleFrame annotation(
    Placement(transformation(origin = {32, -180},  extent = {{16, -16}, {-16, 16}}, rotation = -90),
    iconTransformation(origin = {0, 50}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b leftCP annotation(
    Placement(transformation(origin = {-180, -70}, extent = {{16, -16}, {-16, 16}}, rotation = -90),
    iconTransformation(origin = {-180, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rightCP annotation(
    Placement(transformation(origin = {180, -70}, extent = {{16, -16}, {-16, 16}}, rotation = -90),
    iconTransformation(origin = {180, 0}, extent = {{-16, -16}, {16, 16}})));

  // Wheel torque inputs
  Modelica.Mechanics.Rotational.Interfaces.Flange_b leftTorque annotation(
    Placement(transformation(origin = {-180, 50}, extent = {{-10, -10}, {10, 10}}),
    iconTransformation(origin = {-180, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b rightTorque annotation(
    Placement(transformation(origin = {180, 50}, extent = {{-10, -10}, {10, 10}}),
    iconTransformation(origin = {180, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange if steerable annotation(
    Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

protected
  constant Boolean showBaseIcon = true "Show generic axle artwork when no concrete icon is supplied";

  annotation(
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {
      Line(
        visible = exposeWheelBearings,
        points = {{-180, 90}, {-160, 90}, {-160, 76}, {-120, 76}, {-120, 65}},
        color = {95, 95, 95}),
      Line(
        visible = exposeWheelBearings,
        points = {{180, 90}, {160, 90}, {160, 76}, {120, 76}, {120, 65}},
        color = {95, 95, 95}),
      Text(visible = showBaseIcon, extent = {{-160, 160}, {160, 140}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0),
      Line(visible = showBaseIcon, points = {{-180, 50}, {180, 50}}, color = {95, 95, 95}, thickness = 1),
      Line(visible = showBaseIcon and steerable, points = {{0, 100}, {0, 50}}, color = {95, 95, 95}, thickness = 1),
      Line(visible = showBaseIcon, points = {{-120, 10}, {-120, 0}, {-180, 0}}, color = {95, 95, 95}),
      Line(visible = showBaseIcon, points = {{120, 10}, {120, 0}, {180, 0}}, color = {95, 95, 95}),
      Rectangle(visible = showBaseIcon, extent = {{-140, 90}, {-100, 10}}, lineColor = {71, 71, 71}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, radius = 5),
      Rectangle(visible = showBaseIcon, extent = {{100, 90}, {140, 10}}, lineColor = {71, 71, 71}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, radius = 5)}),
    Documentation(info = "<html>
<p>
An axle attaches to the sprung structure through <code>axleFrame</code>,
exposes wheel-axis torque connections and raw left/right contact-patch frames,
and accepts steering through <code>steerFlange</code>. When steerable is false, the axle owns its steering restraint and transmits
its reactions to axleFrame. The chassis or rig owns contact-patch fixtures.
</p>
<p>
This boundary contains no wishbone geometry, spring actuation, tire model, mass
record, or initialization coordinates. Concrete architectures own those details.
Enable <code>exposeWheelBearings</code> to attach outboard caliper supports to <code>leftBearing</code>/<code>rightBearing</code>. These non-spinning frames follow the upright at the wheel axis; attach the rotor to <code>leftTorque</code>/<code>rightTorque</code>. Inboard calipers instead use the driveline housing-side brake ports. The leftTorque/rightTorque ports are wheel-axis connections. Shaft transport and inboard brake reactions belong to the driveline or test fixture, using the wheel bearing frames and sprung axleFrame.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-180, -180}, {180, 140}}, preserveAspectRatio = true)));
end AxleBase;
