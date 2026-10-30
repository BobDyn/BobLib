within BobLibTest.Drivelines.Internal;

model TestDifferential1D

  BobLib.Drivelines.Internal.Differential1D dut(pDifferential(slidingFrictionRatio = 0.85)) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque drive(tau_constant = 200) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed left(
    exact = true,
    w_ref = 10) annotation(Placement(transformation(origin = {60, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Mechanics.Rotational.Sources.Speed right(
    exact = true,
    w_ref = 20) annotation(Placement(transformation(origin = {60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

equation
  connect(drive.flange, dut.shaft_in) annotation(Line(points = {{-50, 0}, {-10, 0}}, color = {0, 0, 0}));
  connect(left.flange, dut.shaft_left) annotation(Line(points = {{50, 30}, {47, 30}, {47, 4}, {10, 4}}, color = {0, 0, 0}));
  connect(right.flange, dut.shaft_right) annotation(Line(points = {{50, -30}, {47, -30}, {47, -4}, {10, -4}}, color = {0, 0, 0}));
  assert(abs(der(dut.shaft_in.phi)-15) < 1e-8, "Carrier speed is the output average");
  assert(abs(dut.shaft_in.tau+dut.shaft_left.tau+dut.shaft_right.tau) < 1e-8, "Differential torque balance");
  assert(abs(dut.T_lock-46.75) < 1e-6 and dut.lossPower >= 0, "Clutch capacity and dissipation");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 80}}, preserveAspectRatio = true)));
end TestDifferential1D;
