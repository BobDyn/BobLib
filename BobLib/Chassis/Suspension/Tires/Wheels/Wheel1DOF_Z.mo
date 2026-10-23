within BobLib.Chassis.Suspension.Tires.Wheels;

model Wheel1DOF_Z

  // Record parameters

  extends BobLib.Chassis.Suspension.Tires.Wheels.PartialWheel(wheelJ = 0.01);

  replaceable Modelica.Mechanics.Translational.Components.SpringDamper springDamper(
    c = pWheel.vertical.wheelC,
    d = pWheel.vertical.wheelD,
    s_rel0 = pWheel.geometry.R0) constrainedby Modelica.Mechanics.Translational.Interfaces.PartialCompliant annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-30, -46}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(springDamper.flange_a, prismatic_z.support) annotation(
    Line(points = {{-30, -36}, {-6, -36}}, color = {0, 127, 0}));
  connect(springDamper.flange_b, prismatic_z.axis) annotation(
    Line(points = {{-30, -56}, {-30, -66}, {-14, -66}, {-14, -48}, {-6, -48}}, color = {0, 127, 0}));
  connect(inertia.flange_a, roadReaction.flange_b) annotation(
    Line(visible = not includeRotorEffects, points = {{30, 30}, {14, 30}, {14, 10}}));
  connect(spatialInertia.flange_a, roadReaction.flange_b) annotation(
    Line(visible = includeRotorEffects, points = {{30, 30}, {14, 30}, {14, 10}}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Icon(graphics = {Ellipse(fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-15, -15}, {15, 15}}), Line(origin = {0, -81}, points = {{0, -19}, {0, 19}}, color = {255, 255, 255}, thickness = 5), Line(origin = {-8.97, -80}, points = {{-1.02758, -20}, {-1.02758, -6}, {-5.02758, -4}, {2.97242, 0}, {-5.02758, 4}, {2.97242, 8}, {-1.02758, 10}, {-1.02758, 20}}, color = {0, 170, 0}, thickness = 2), Line(origin = {10, -91}, points = {{0, -9}, {0, 9}}, color = {0, 170, 0}, thickness = 2), Line(origin = {10, -82}, points = {{-5, 0}, {5, 0}}, color = {0, 170, 0}, thickness = 2), Line(origin = {10, -81}, points = {{-8, -5}, {-8, 5}, {8, 5}, {8, -5}}, color = {0, 170, 0}, thickness = 2), Line(origin = {10, -68}, points = {{0, -8}, {0, 8}}, color = {0, 170, 0}, thickness = 2)}),
    Documentation(info = "<html>
<p>
Model <code>Wheel1DOF_Z</code> represents a wheel physics variant with 1DOF Z behavior.
</p>
<p>
It supplies wheel-center motion and contact inputs to the tire model while keeping the tire force equations independent of wheel DOF choice.
</p>
</html>"));
end Wheel1DOF_Z;
