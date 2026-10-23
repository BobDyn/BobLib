within BobLibTest.Chassis.Suspension.Tires.MF52.SlipModel;

model TestNoSlip

  BobLib.Chassis.Suspension.Tires.MF52.SlipModel.NoSlip dut(
    Vx = 10,
    Vy = 1,
    omega = 48,
    R0 = 0.25,
    Fz = 1000,
    gamma = 0) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  assert(abs(dut.kappa) < 1e-12 and abs(dut.alpha) < 1e-12, "NoSlip suppresses longitudinal and lateral slip");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}}, preserveAspectRatio = true)));
end TestNoSlip;
