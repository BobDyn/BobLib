within BobLibTest.Engines.Internal;

model TestSimpleICEngineCore

  BobLib.Engines.Internal.SimpleICEngineCore dut(
    throttle = 0.5,
    w = 0) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  assert(abs(dut.tau - 60) < 1e-10, "Half throttle at rest produces half peak torque");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}}, preserveAspectRatio = true)));
end TestSimpleICEngineCore;
