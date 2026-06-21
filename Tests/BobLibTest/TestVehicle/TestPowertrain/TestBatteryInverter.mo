within BobLibTest.TestVehicle.TestPowertrain;

model TestBatteryInverter

  import SI = Modelica.Units.SI;

  // Battery
  BobLib.EnergyStorage.Internal.TheveninBatteryPack batt(Ns = 140, Np = 4, SOC_start = 1.0) annotation(
    Placement(transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}})));

  // Inverter
  BobLib.PowerElectronics.InverterDC inv annotation(
    Placement(transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Interfaces.ElectricMotorControlBus electricMotorControlBus annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}})));

  // Power command
  Modelica.Blocks.Sources.Step P_step(height = 80e3, // 80 kW
  startTime = 1.0) annotation(
    Placement(transformation(origin = {-30, 60}, extent = {{-10, -10}, {10, 10}})));

  // Electrical reference
  Modelica.Electrical.Analog.Basic.Ground g annotation(
    Placement(transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}})));
  BobLib.ElectricDrives.Motor motor annotation(
    Placement(transformation(origin = {38, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1) annotation(
    Placement(transformation(origin = {72, 40}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(controlBus, inv.controlBus) annotation(
    Line(points = {{-30, 30}, {-10, 30}, {-10, 46}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-30, 30}, {16, 30}, {16, 34}, {28, 34}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus) annotation(
    Line(points = {{-30, 30}, {-30, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(P_step.y, electricMotorControlBus.powerRequest) annotation(
    Line(points = {{-19, 60}, {-30, 60}, {-30, 40}}, color = {0, 0, 255}));
  connect(g.p, batt.p) annotation(
    Line(points = {{-30, -30}, {-30, -10}, {-10, -10}}, color = {0, 0, 255}));
  connect(inv.p, batt.p) annotation(
    Line(points = {{-10, 40}, {-20, 40}, {-20, -10}, {-10, -10}}, color = {0, 0, 255}));
  connect(inv.n, batt.n) annotation(
    Line(points = {{10, 40}, {20, 40}, {20, -10}, {10, -10}}, color = {0, 0, 255}));
  connect(inv.motor_p, motor.pin_p) annotation(
    Line(points = {{10, 44}, {18, 44}, {18, 44.4}, {28, 44.4}}, color = {0, 0, 255}));
  connect(inv.motor_n, motor.pin_n) annotation(
    Line(points = {{10, 36}, {18, 36}, {18, 42}, {28, 42}}, color = {0, 0, 255}));
  connect(motor.shaft_b.flange, inertia.flange_a) annotation(
    Line(points = {{48, 40}, {62, 40}}));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Interval = 0.002, Tolerance = 1e-06),
    __OpenModelica_simulationFlags(s = "cvode", lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", variableFilter = ".*"),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end TestBatteryInverter;
