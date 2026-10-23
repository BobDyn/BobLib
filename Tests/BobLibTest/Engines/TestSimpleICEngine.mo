within BobLibTest.Engines;

model TestSimpleICEngine

  BobLib.Engines.SimpleICEngine dut(engineSpeed_start = 0) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant throttle(k = 0.5) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(throttle.y, dut.controlBus.driverBus.acceleratorPedalPosition) annotation(Line(points = {{-49, 0}, {-47, 0}, {-47, 6}, {-10, 6}}, color = {0, 0, 127}));
  assert(dut.w >= -1e-8, "Positive throttle accelerates the unloaded engine forwards");

  when terminal() then
    assert(dut.w > 0, "Positive throttle must accelerate the unloaded engine");
  end when;
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -40}, {40, 40}}, preserveAspectRatio = true)));
end TestSimpleICEngine;
