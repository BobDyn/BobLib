within BobLibTest.Drivelines.Internal;

model TestLockedDifferential1D

  BobLib.Drivelines.Internal.LockedDifferential1D dut annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque drive(tau_constant = 200) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia left(
    J = 1,
    w(start = 0, fixed = true),
    phi(
      start = 0,
      fixed = true)) annotation(Placement(transformation(origin = {60, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia right(J = 1) annotation(Placement(transformation(origin = {60, -30}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(drive.flange, dut.shaft_in) annotation(Line(points = {{-50, 0}, {-10, 0}}, color = {0, 0, 0}));
  connect(left.flange_a, dut.shaft_left) annotation(Line(points = {{50, 30}, {47, 30}, {47, 4}, {10, 4}}, color = {0, 0, 0}));
  connect(right.flange_a, dut.shaft_right) annotation(Line(points = {{50, -30}, {47, -30}, {47, -4}, {10, -4}}, color = {0, 0, 0}));
  assert(abs(left.w-right.w) < 1e-8, "Spool equal speeds");
  assert(abs(left.w-100*time) < 1e-6, "Spool acceleration and inertia");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 80}}, preserveAspectRatio = true)));
end TestLockedDifferential1D;
