within BobLibTest.EnergyStorage;
model TestBatteryPack

  BobLib.EnergyStorage.BatteryPack dut(
    pBattery(SOC_start = 0.8),
    includeGround = false) annotation(Placement(transformation(origin = {-40, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Electrical.Analog.Basic.Resistor load(R = 100) annotation(Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(dut.pin_p, load.p) annotation(Line(points = {{-20, 12}, {-17, 12}, {-17, 16}, {40, 16}, {40, 10}}, color = {0, 0, 255}));
  connect(dut.pin_n, load.n) annotation(Line(points = {{-20, -12}, {-17, -12}, {-17, -14}, {40, -14}, {40, -10}}, color = {0, 0, 255}));
  connect(load.n, ground.p) annotation(Line(points = {{40, -10}, {40, -24}, {0, -24}, {0, -30}}, color = {0, 0, 255}));
  assert(dut.v>0 and dut.i>0 and dut.SOC<=0.8+1e-10, "Loaded pack must discharge");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-100, -80}, {80, 60}}, preserveAspectRatio = true)));
end TestBatteryPack;
