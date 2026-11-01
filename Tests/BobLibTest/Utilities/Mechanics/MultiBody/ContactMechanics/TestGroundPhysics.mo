within BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics;

model TestGroundPhysics

  import Modelica.Math.Vectors.normalize;

  // Force sensor
  Modelica.Mechanics.MultiBody.Sensors.CutForce cut_force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

protected

  // Set gravity
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(Placement(transformation(origin = {-210, -110}, extent = {{-10, -10}, {10, 10}})));

  // Define ground reference
  Modelica.Mechanics.MultiBody.Parts.Fixed ground_reference(r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Define ground
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Define mass above ground
  Modelica.Mechanics.MultiBody.Parts.Body body(
    r_CM = {0, 0, 0},
    m = 1,
    sphereDiameter = 0.05,
    r_0(start = {0, 0, 0.25}, each fixed = true)) annotation(
    Placement(transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Define force with x and y component
  Modelica.Blocks.Sources.RealExpression e_f[3](y = normalize({1, 1, 0})) annotation(
    Placement(transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
    Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(e_f.y, force.force) annotation(
    Line(points = {{-59, 50}, {-42, 50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(force.frame_b, body.frame_a) annotation(
    Line(points = {{-20, 50}, {0, 50}, {0, 60}}, color = {95, 95, 95}));
  connect(ground_reference.frame_b, ground.frame_a) annotation(
    Line(points = {{0, -60}, {-0, -40}}, color = {95, 95, 95}));
  connect(ground.frame_b, cut_force.frame_a) annotation(
    Line(points = {{-10, -30}, {-13, -30}, {-13, -4}, {-0, -4}, {-0, 0}}, color = {95, 95, 95}));
  connect(cut_force.frame_b, body.frame_a) annotation(
    Line(points = {{0, 20}, {-0, 60}}, color = {95, 95, 95}));
  assert(abs(body.a_0[1] - 1/sqrt(2)/body.m) < 1e-7 and
    abs(body.a_0[2] - 1/sqrt(2)/body.m) < 1e-7,
    "The frictionless ground must not resist the applied horizontal force");
  assert(body.a_0[3] >= -world.g - 1e-7,
    "Unilateral ground contact must not pull the body downward");
  annotation(
    experiment(StartTime = 0, StopTime = 2));
  annotation(Diagram(coordinateSystem(extent = {{-220, -120}, {120, 120}}, preserveAspectRatio = true)));
end TestGroundPhysics;
