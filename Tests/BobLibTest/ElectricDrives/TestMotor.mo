within BobLibTest.ElectricDrives;
model TestMotor "Independent averaged motor load under motoring and regeneration"

  BobLib.ElectricDrives.Motor dut annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

  Modelica.Electrical.Analog.Sources.ConstantVoltage supply(V = 500) annotation(
    Placement(transformation(origin = {-60, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(transformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression power(y = if time < 0.02 then 1000 else -500) annotation(
    Placement(transformation(origin = {70, 40}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed speed(exact = true, w_ref = 200) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(supply.p, dut.pin_p) annotation(Line(points = {{-60, 60}, {-60, 70}, {12, 70}, {12, 20}}, color = {0, 0, 255}));
  connect(supply.n, dut.pin_n) annotation(Line(points = {{-60, 40}, {-60, 30}, {-12, 30}, {-12, 20}}, color = {0, 0, 255}));
  connect(ground.p, supply.n) annotation(Line(points = {{-90, 30}, {-60, 30}, {-60, 40}}, color = {0, 0, 255}));
  connect(power.y, dut.controlBus.electricMotorControlBus.availablePower) annotation(Line(points = {{59, 40}, {35, 40}, {35, 12}, {20, 12}}, color = {0, 0, 127}));
  connect(dut.shaft_b.flange, speed.flange) annotation(Line(points = {{20, 0}, {50, 0}}, color = {0, 0, 0}));
  assert(abs(dut.P_elec - power.y) < 1e-6, "Motor did not consume allocated signed power");
  assert(abs(dut.pin_p.i + dut.pin_n.i) < 1e-9, "Motor violates current conservation");
  annotation(experiment(StopTime = 0.05, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-120, -80}, {100, 100}}, preserveAspectRatio = true)));
end TestMotor;
