within BobLib.Tests.TestVehicle.TestPowertrain;

model TestPowertrain
  import SI = Modelica.Units.SI;
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {120, 50}, extent = {{-10, -10}, {10, 10}})));
  // Battery
  BobLib.Vehicle.Powertrain.Battery.BatteryPack batt(Ns = 140, Np = 4, SOC_start = 1.0) annotation(
    Placement(transformation(origin = {-80, -20}, extent = {{-10, -10}, {10, 10}})));
  // Inverter
  BobLib.Vehicle.Electronics.PowerElectronics.InverterDC inv annotation(
    Placement(transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}})));
  // Power command
  Modelica.Blocks.Sources.Step P_step(height = 80e3,  // 80 kW
  startTime = 1.0) annotation(
    Placement(transformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}})));
  // Electrical reference
  Modelica.Electrical.Analog.Basic.Ground g annotation(
    Placement(transformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Powertrain.Drivetrain.Motor motor annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor annotation(
    Placement(transformation(origin = {60, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.Rotational.Components.Inertia leftWheelInertia(
    J = 1,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Powertrain.Drivetrain.Differential differential(
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
  Modelica.Mechanics.MultiBody.Parts.Fixed differentialSupport annotation(
    Placement(transformation(origin = {20, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
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
  connect(P_step.y, inv.P_req) annotation(
    Line(points = {{-99, 50}, {-80, 50}, {-80, 42}}, color = {0, 0, 255}));
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
  connect(differentialSupport.frame_b, differential.mountFrame) annotation(
    Line(points = {{20, 32}, {20, 10}}, color = {95, 95, 95}));
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
