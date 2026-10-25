within BobLibTest.Chassis.Suspension.Components;

model TestForceOnlyRod

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedA(r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedB(r = {1, 0, 0}) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  BobLib.Chassis.Suspension.Components.ForceOnlyRod rod(
    r_a = {0, 0, 0},
    r_b = {1, 0, 0}) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(fixedA.frame_b, rod.frame_a) annotation(
    Line(points = {{-50, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(fixedB.frame_b, rod.frame_b) annotation(
    Line(points = {{50, 0}, {20, 0}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 60}}, preserveAspectRatio = true)));
end TestForceOnlyRod;
