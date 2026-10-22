within BobLibTest.Transmissions;

model TestFixedRatioTransmission

  BobLib.Transmissions.FixedRatioTransmission dut(pTransmission(gearRatio = 3)) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed speed(
    exact = true,
    w_ref = 30) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque load(tau_constant = -10) annotation(Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Blocks.Interfaces.IntegerOutput engagedGear annotation(
    Placement(transformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}})));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(dut.controlBus.transmissionControlBus.currentGear, engagedGear) annotation(
    Line(points = {{-10, 6}, {-20, 6}, {-20, 30}, {100, 30}}, color = {255, 127, 0}));
  assert(engagedGear == 1, "Fixed reducer must publish engaged gear 1");
  connect(speed.flange, dut.engineFlange.flange) annotation(Line(points = {{-50, 0}, {-10, 0}}, color = {0, 0, 0}));
  connect(dut.drivelineFlange.flange, load.flange) annotation(Line(points = {{10, 0}, {50, 0}}, color = {0, 0, 0}));
  assert(abs(der(dut.drivelineFlange.flange.phi) - 10) < 1e-8, "Gear speed ratio");
  assert(abs(dut.engineFlange.flange.tau*30 + dut.drivelineFlange.flange.tau*10) < 1e-8, "Ideal gear power balance");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -40}, {100, 40}}, preserveAspectRatio = true)));
end TestFixedRatioTransmission;
