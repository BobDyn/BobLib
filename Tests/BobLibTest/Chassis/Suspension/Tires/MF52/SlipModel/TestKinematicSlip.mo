within BobLibTest.Chassis.Suspension.Tires.MF52.SlipModel;

model TestKinematicSlip

  BobLib.Chassis.Suspension.Tires.MF52.SlipModel.KinematicSlip dut(
    Vx = 10,
    Vy = 1,
    omega = 48,
    R0 = 0.25,
    Fz = 1000,
    gamma = 0) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  assert(dut.kappa > 0 and dut.alpha < 0, "Driven wheel has positive longitudinal slip and opposing slip angle");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}}, preserveAspectRatio = true)));
end TestKinematicSlip;
