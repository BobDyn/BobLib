within BobLibTest.Drivelines;

model TestPowertrain

  import Modelica.Units.SI;

  // Battery
  BobLib.EnergyStorage.Internal.TheveninBatteryPack batt(pBattery(Ns = 140, Np = 4, SOC_start = 1.0)) annotation(
    Placement(transformation(origin = {-140, 30}, extent = {{-10, -6}, {10, 6}}, rotation = 270)));

  // Inverter
  BobLib.PowerElectronics.InverterDC inv annotation(
    Placement(transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {-150, 80}, extent = {{-10, -10}, {10, 10}})));
  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus annotation(
    Placement(transformation(origin = {-110, 80}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));

  // Power command
  Modelica.Blocks.Sources.Step P_step(height = 80e3, // 80 kW
  startTime = 1.0) annotation(
    Placement(transformation(origin = {-70, 80}, extent = {{10, -10}, {-10, 10}})));

  // Electrical reference
  Modelica.Electrical.Analog.Basic.Ground g annotation(
    Placement(transformation(origin = {-140, -10}, extent = {{-10, -10}, {10, 10}})));
  BobLib.ElectricDrives.Motor motor annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation(
    Placement(transformation(origin = {60, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.Rotational.Components.Inertia leftWheelInertia(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Drivelines.Internal.Differential1D differential(
    pDifferential(
      driveSideTorqueSign = 1,
      preloadTorque = 25,
      driveLockingFraction = 0.40,
      coastLockingFraction = 0.20,
      slidingFrictionRatio = 0.80),
    differentialArchitecture = BobLib.Drivelines.Types.DifferentialArchitecture.ClutchLSD) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 3.31, useSupport = false) annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor1 annotation(
    Placement(transformation(origin = {60, -20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia rightWheelInertia(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {90, -20}, extent = {{-10, -10}, {10, 10}})));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-170, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  assert(abs(leftWheelInertia.w - rightWheelInertia.w) < 1e-8,
    "Symmetric wheel inertias must accelerate together");
  assert(abs(differential.T_left_drive - differential.T_right_drive) < 1e-8,
    "Symmetric wheel inertias must receive equal torque");
  connect(controlBus, inv.controlBus) annotation(
    Line(points = {{-150, 80}, {-150, 60}, {-86, 60}, {-86, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-150, 80}, {-150, 60}, {-25, 60}, {-25, 6}, {-30, 6}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus) annotation(
    Line(points = {{-150, 80}, {-110, 80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(P_step.y, electricMotorControlBus.powerRequest) annotation(
    Line(points = {{-81, 80}, {-110, 80}}, color = {0, 0, 255}));
  connect(g.p, batt.p) annotation(
    Line(points = {{-140, 0}, {-160, 0}, {-160, 40}, {-140, 40}}, color = {0, 0, 255}));
  connect(inv.p, batt.p) annotation(
    Line(points = {{-90, 36.2}, {-110, 36.2}, {-110, 40}, {-140, 40}}, color = {0, 0, 255}));
  connect(inv.n, batt.n) annotation(
    Line(points = {{-90, 24}, {-110, 24}, {-110, 20}, {-140, 20}}, color = {0, 0, 255}));
  connect(inv.motor_p, motor.pin_p) annotation(
    Line(points = {{-70, 36}, {-34, 36}, {-34, 10}}, color = {0, 0, 255}));
  connect(inv.motor_n, motor.pin_n) annotation(
    Line(points = {{-70, 24}, {-46, 24}, {-46, 10}}, color = {0, 0, 255}));
  connect(torqueSensor.flange_b, leftWheelInertia.flange_a) annotation(
    Line(points = {{70, 20}, {80, 20}}, color = {0, 0, 0}));
  connect(motor.shaft_b.flange, idealGear.flange_a) annotation(
    Line(points = {{-30, 0}, {-20, 0}}, color = {0, 0, 0}));
  connect(idealGear.flange_b, differential.shaft_in) annotation(
    Line(points = {{0, 0}, {10, 0}}, color = {0, 0, 0}));
  connect(torqueSensor1.flange_b, rightWheelInertia.flange_a) annotation(
    Line(points = {{70, -20}, {80, -20}}, color = {0, 0, 0}));
  connect(torqueSensor1.flange_a, differential.shaft_right) annotation(
    Line(points = {{50, -20}, {47, -20}, {47, -4}, {30, -4}}, color = {0, 0, 0}));
  connect(torqueSensor.flange_a, differential.shaft_left) annotation(
    Line(points = {{50, 20}, {47, 20}, {47, 4}, {30, 4}}, color = {0, 0, 0}));
annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
  Diagram(coordinateSystem(extent = {{-180, -80}, {140, 100}}, preserveAspectRatio = true)));
end TestPowertrain;
