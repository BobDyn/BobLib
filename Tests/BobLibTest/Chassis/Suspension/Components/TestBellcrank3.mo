within BobLibTest.Chassis.Suspension.Components;

model TestBellcrank3

  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-130, -90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed mountFixed(r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body pickup1Mass(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4) annotation(
    Placement(transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Mechanics.MultiBody.Parts.Body pickup2Mass(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body pickup3Mass(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4) annotation(
    Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  BobLib.Chassis.Suspension.Components.Bellcrank3 bellcrank(
    pivot = {0, 0, 0},
    pivotAxis = {0, 0, 1},
    pickup_1 = {0.1, 0, 0},
    pickup_2 = {0.1, 0.1, 0},
    pickup_3 = {0, 0.1, 0},
    linkDiameter = 0.020,
    jointDiameter = 0.030) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(mountFixed.frame_b, bellcrank.mountFrame) annotation(
    Line(points = {{-50, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(bellcrank.pickupFrame1, pickup1Mass.frame_a) annotation(
    Line(points = {{0, -20}, {0, -40}}, color = {95, 95, 95}));
  connect(bellcrank.pickupFrame2, pickup2Mass.frame_a) annotation(
    Line(points = {{20, 0}, {50, 0}}, color = {95, 95, 95}));
  connect(bellcrank.pickupFrame3, pickup3Mass.frame_a) annotation(
    Line(points = {{0, 20}, {0, 40}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}}, preserveAspectRatio = true)));
end TestBellcrank3;
