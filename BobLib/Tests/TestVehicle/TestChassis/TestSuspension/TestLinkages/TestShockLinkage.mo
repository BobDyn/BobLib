within BobLib.Tests.TestVehicle.TestChassis.TestSuspension.TestLinkages;

model TestShockLinkage
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-60, -60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedA(r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedB(r = {1, 0, 0}) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage shock(
    r_a = {0, 0, 0},
    r_b = {1, 0, 0},
    n_a = {0, 1, 0},
    n_b = {0, 1, 0},
    s_0 = 1,
    springTable = [-0.1, -100; 0, 0; 0.1, 100],
    damperTable = [-1, -50; 0, 0; 1, 50],
    linkDiameter = 0.020,
    jointDiameter = 0.030) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(fixedA.frame_b, shock.frame_a) annotation(
    Line(points = {{-50, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(fixedB.frame_b, shock.frame_b) annotation(
    Line(points = {{50, 0}, {20, 0}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestShockLinkage;
