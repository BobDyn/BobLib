within BobLibTest.Transmissions.Internal;

model TestFixedRatioGear

  BobLib.Transmissions.Internal.FixedRatioGear dut(gearRatio = 3) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed speed(
    exact = true,
    w_ref = 30) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque load(tau_constant = -10) annotation(Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

equation
  connect(speed.flange, dut.inputFlange) annotation(Line(points = {{-50, 0}, {-10, 0}}, color = {0, 0, 0}));
  connect(dut.outputFlange, load.flange) annotation(Line(points = {{10, 0}, {50, 0}}, color = {0, 0, 0}));
  assert(abs(der(dut.outputFlange.phi) - 10) < 1e-8, "Gear speed ratio");
  assert(abs(dut.inputFlange.tau*30 + dut.outputFlange.tau*10) < 1e-8, "Ideal gear power balance");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -40}, {100, 40}}, preserveAspectRatio = true)));
end TestFixedRatioGear;
