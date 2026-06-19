within BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain;

model TestPowertrain
  import SI = Modelica.Units.SI;
  // Battery
  BobLibVehicleInterfaces.EnergyStorage.Internal.TheveninBatteryPack batt(Ns = 140, Np = 4, SOC_start = 1.0) annotation(
    Placement(transformation(origin = {-80, -20}, extent = {{-10, -10}, {10, 10}})));
  // Inverter
  BobLibVehicleInterfaces.PowerElectronics.InverterDC inv annotation(
    Placement(transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {-110, 20}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Interfaces.ElectricMotorControlBus electricMotorControlBus annotation(
    Placement(transformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}})));
  // Power command
  Modelica.Blocks.Sources.Step P_step(height = 80e3,  // 80 kW
  startTime = 1.0) annotation(
    Placement(transformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}})));
  // Electrical reference
  Modelica.Electrical.Analog.Basic.Ground g annotation(
    Placement(transformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.ElectricDrives.Internal.PowerLimitedMotor motor annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation(
    Placement(transformation(origin = {60, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.Rotational.Components.Inertia leftWheelInertia(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Drivelines.Internal.Differential1D differential(
    driveSideTorqueSign = 1,
    T_preload = 25,
    lockFractionAccel = 0.40,
    lockFractionDecel = 0.20,
    T_capacity_max = 250,
    clutchEffectiveRadius = 1.0,
    kineticFrictionRatio = 0.80,
    w_transition = 1.5,
    c_viscous = 0.05) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 3.31, useSupport = false)  annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1 annotation(
    Placement(transformation(origin = {60, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Inertia rightWheelInertia(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {90, -20}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(controlBus, inv.controlBus) annotation(
    Line(points = {{-110, 20}, {-90, 20}, {-90, 36}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus) annotation(
    Line(points = {{-110, 20}, {-110, 30}}, color = {255, 204, 51}, thickness = 0.5));
  connect(P_step.y, electricMotorControlBus.powerRequest) annotation(
    Line(points = {{-99, 50}, {-110, 50}, {-110, 30}}, color = {0, 0, 255}));
  connect(g.p, batt.p) annotation(
    Line(points = {{-110, -40}, {-110, -20}, {-90, -20}}, color = {0, 0, 255}));
  connect(inv.p, batt.p) annotation(
    Line(points = {{-90, 30}, {-100, 30}, {-100, -20}, {-90, -20}}, color = {0, 0, 255}));
  connect(inv.n, batt.n) annotation(
    Line(points = {{-70, 30}, {-60, 30}, {-60, -20}, {-70, -20}}, color = {0, 0, 255}));
  connect(inv.P_out, motor.P_elec) annotation(
    Line(points = {{-80, 19}, {-80, -1}, {-52, -1}}, color = {0, 0, 127}));
  connect(torqueSensor.flange_b, leftWheelInertia.flange_a) annotation(
    Line(points = {{80, 20}, {70, 20}}));
  connect(motor.shaft, idealGear.flange_a) annotation(
    Line(points = {{-30, 0}, {-20, 0}}));
  connect(idealGear.flange_b, differential.shaft_in) annotation(
    Line(points = {{0, 0}, {10, 0}}));
  connect(torqueSensor1.flange_b, rightWheelInertia.flange_a) annotation(
    Line(points = {{80, -20}, {70, -20}}));
  connect(torqueSensor1.flange_a, differential.shaft_right) annotation(
    Line(points = {{50, -20}, {40, -20}, {40, -4}, {30, -4}}));
  connect(torqueSensor.flange_a, differential.shaft_left) annotation(
    Line(points = {{50, 20}, {40, 20}, {40, 4}, {30, 4}}));
annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
  Diagram(coordinateSystem(extent = {{-120, 60}, {140, -60}})));
end TestPowertrain;
