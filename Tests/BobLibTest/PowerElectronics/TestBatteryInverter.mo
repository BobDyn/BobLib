within BobLibTest.PowerElectronics;

model TestBatteryInverter

  import Modelica.Units.SI;

  // Battery
  BobLib.EnergyStorage.Internal.TheveninBatteryPack batt(pBattery(Ns = 140, Np = 4, SOC_start = 1.0)) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -6}, {10, 6}}, rotation = 270)));

  // Inverter
  BobLib.PowerElectronics.InverterDC inv annotation(
    Placement(transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  BobLib.Utilities.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));

  // Power command
  Modelica.Blocks.Sources.Step P_step(height = 80e3, // 80 kW
  startTime = 1.0) annotation(
    Placement(transformation(origin = {-60, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  // Electrical reference
  Modelica.Electrical.Analog.Basic.Ground g annotation(
    Placement(transformation(origin = {-60, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  BobLib.ElectricDrives.Motor motor annotation(
    Placement(transformation(origin = {38, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.Rotational.Components.Inertia inertia(J = 1) annotation(
    Placement(transformation(origin = {38, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(controlBus, inv.controlBus) annotation(
    Line(points = {{-90, 40}, {-90, 70}, {-6, 70}, {-6, 50}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-90, 40}, {-90, 70}, {32, 70}, {32, 50}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus) annotation(
    Line(points = {{-90, 40}, {-90, 70}}, color = {255, 204, 51}, thickness = 0.5));
  connect(P_step.y, electricMotorControlBus.powerRequest) annotation(
    Line(points = {{-71, 90}, {-90, 90}, {-90, 70}}, color = {0, 0, 255}));
  connect(g.p, batt.p) annotation(
    Line(points = {{-60, 20}, {-80, 20}, {-80, 50}, {-60, 50}}, color = {0, 0, 255}));
  connect(inv.p, batt.p) annotation(
    Line(points = {{-10, 46.2}, {-30, 46.2}, {-30, 50}, {-60, 50}}, color = {0, 0, 255}));
  connect(inv.n, batt.n) annotation(
    Line(points = {{-10, 34}, {-30, 34}, {-30, 30}, {-60, 30}}, color = {0, 0, 255}));
  connect(inv.motor_p, motor.pin_p) annotation(
    Line(points = {{10, 46}, {28, 46}}, color = {0, 0, 255}));
  connect(inv.motor_n, motor.pin_n) annotation(
    Line(points = {{10, 34}, {28, 34}}, color = {0, 0, 255}));
  connect(motor.shaft_b.flange, inertia.flange_a) annotation(
    Line(points = {{38, 50}, {38, 70}}, color = {0, 0, 0}));
  assert(inertia.J*inertia.w*der(inertia.w) <= batt.P + 1e-5,
    "Mechanical load power cannot exceed battery terminal power during motoring");
  assert(time < 2 or inertia.w > 0,
    "The positive power step must accelerate the connected mechanical load");
  annotation(
    experiment(StartTime = 0, StopTime = 10, Interval = 0.002, Tolerance = 1e-06),
    __OpenModelica_simulationFlags(s = "cvode", lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", variableFilter = ".*"),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
  annotation(Diagram(coordinateSystem(extent = {{-120, -80}, {120, 100}}, preserveAspectRatio = true)));
end TestBatteryInverter;
