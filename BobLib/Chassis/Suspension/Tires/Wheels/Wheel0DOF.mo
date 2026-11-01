within BobLib.Chassis.Suspension.Tires.Wheels;

model Wheel0DOF

  extends BobLib.Chassis.Suspension.Tires.Wheels.PartialWheel(
    wheelJ = 0.01,
    prismatic_z(stateSelect = StateSelect.never));

  Modelica.Mechanics.Translational.Components.Rod fixedRadius(L = pWheel.geometry.R0) annotation(
    Placement(transformation(origin = {-30, -46}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(fixedRadius.flange_a, prismatic_z.support) annotation(
    Line(points = {{-30, -36}, {-6, -36}}, color = {0, 127, 0}));
  connect(fixedRadius.flange_b, prismatic_z.axis) annotation(
    Line(points = {{-30, -56}, {-30, -66}, {-14, -66}, {-14, -48}, {-6, -48}}, color = {0, 127, 0}));
  connect(inertia.flange_a, roadReaction.flange_b) annotation(
    Line(visible = not includeRotorEffects, points = {{30, 30}, {14, 30}, {14, 10}}));
  connect(spatialInertia.flange_a, roadReaction.flange_b) annotation(
    Line(visible = includeRotorEffects, points = {{30, 30}, {14, 30}, {14, 10}}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Icon(graphics = {Ellipse(fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-15, -15}, {15, 15}}), Line(origin = {0, -81}, points = {{0, -19}, {0, 19}}, thickness = 5)}),
    Documentation(info = "<html>
<p>
Model <code>Wheel0DOF</code> represents a wheel physics variant with 0DOF behavior.
</p>
<p>
It supplies wheel-center motion and contact inputs to the tire model while keeping the tire force equations independent of wheel DOF choice.
</p>
</html>"));
end Wheel0DOF;
