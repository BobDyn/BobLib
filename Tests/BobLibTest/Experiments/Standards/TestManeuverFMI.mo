within BobLibTest.Experiments.Standards;
model TestManeuverFMI

  BobLib.Experiments.Standards.ManeuverFMI dut annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.Constant speed(k = 15) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant ay(k = 0) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant loads[4](each k = 1000) annotation(
    Placement(transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(speed.y, dut.speed) annotation(Line(points = {{-49, 40}, {-20, 40}, {-20, 16}, {10, 16}}, color = {0, 0, 127}));
  connect(ay.y, dut.ay) annotation(Line(points = {{-49, 0}, {-30, 0}, {-30, 8}, {10, 8}}, color = {0, 0, 127}));
  connect(loads.y, dut.loads) annotation(Line(points = {{-49, -40}, {-10, -40}, {-10, -4}, {10, -4}}, color = {0, 0, 127}));
  assert(abs(dut.accelerator)<1e-12 and abs(dut.brake)<1e-12, "No speed error must produce no pedal demand");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})));
end TestManeuverFMI;
