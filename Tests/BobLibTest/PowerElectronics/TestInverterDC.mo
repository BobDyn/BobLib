within BobLibTest.PowerElectronics;
model TestInverterDC "Independent averaged voltage supply with a matched resistive load"

  BobLib.PowerElectronics.InverterDC dut annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

  Modelica.Electrical.Analog.Sources.ConstantVoltage supply(V = 500) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
    Placement(transformation(origin = {-70, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Basic.Resistor load(R = 250) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Blocks.Sources.Constant power(k = 1000) annotation(
    Placement(transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(supply.p, dut.p) annotation(Line(points = {{-70, 10}, {-50, 10}, {-50, 12.4}, {-20, 12.4}}, color = {0, 0, 255}));
  connect(supply.n, dut.n) annotation(Line(points = {{-70, -10}, {-50, -10}, {-50, -12}, {-20, -12}}, color = {0, 0, 255}));
  connect(supply.n, ground.p) annotation(Line(points = {{-70, -10}, {-70, -30}}, color = {0, 0, 255}));
  connect(dut.motor_p, load.p) annotation(Line(points = {{20, 12}, {50, 12}, {50, 10}, {70, 10}}, color = {0, 0, 255}));
  connect(dut.motor_n, load.n) annotation(Line(points = {{20, -12}, {50, -12}, {50, -10}, {70, -10}}, color = {0, 0, 255}));
  connect(power.y, dut.controlBus.electricMotorControlBus.powerRequest) annotation(Line(points = {{11, 60}, {25, 60}, {25, 35}, {-12, 35}, {-12, 20}}, color = {0, 0, 127}));
  assert(abs(dut.V_motor - 500) < 1e-6, "Inverter did not supply output voltage");
  assert(abs(load.v*load.i - dut.P_out) < 1e-6, "Matched load differs from allocated power");
  assert(abs(dut.p.i + dut.n.i + dut.motor_p.i + dut.motor_n.i) < 1e-9, "Inverter violates current conservation");
  assert(abs(dut.P_dc - dut.V_motor*dut.I_motor - dut.P_loss) < 1e-6,
    "Inverter violates power conservation with the allocated load");
  annotation(experiment(StopTime = 0.01, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -80}, {100, 100}}, preserveAspectRatio = true)));
end TestInverterDC;
