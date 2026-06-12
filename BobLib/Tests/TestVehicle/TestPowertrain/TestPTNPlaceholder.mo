within BobLib.Tests.TestVehicle.TestPowertrain;

model TestPTNPlaceholder
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Vehicle.Powertrain.PTNPlaceholder ptn annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-20, -8}, {20, 8}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false) annotation(
    Placement(transformation(origin = {0, 34}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Blocks.Sources.Constant torqueCmd(k = 20) annotation(
    Placement(transformation(origin = {-50, -34}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia leftWheel(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia rightWheel(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(fixed.frame_b, ptn.mountFrame) annotation(
    Line(points = {{0, 24}, {0, 8}}, color = {95, 95, 95}));
  connect(torqueCmd.y, ptn.u) annotation(
    Line(points = {{-39, -34}, {0, -34}, {0, -8}}, color = {0, 0, 127}));
  connect(leftWheel.flange_b, ptn.leftFlange) annotation(
    Line(points = {{-40, 0}, {-20, 0}}));
  connect(ptn.rightFlange, rightWheel.flange_a) annotation(
    Line(points = {{20, 0}, {40, 0}}));

annotation(
  experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
  Diagram(coordinateSystem(extent = {{-80, -50}, {80, 70}})));
end TestPTNPlaceholder;
