within BobLib.Tests.TestVehicle.TestPowertrain;

model TestPowertrainBatInvMotDiff
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Vehicle.Powertrain.PowertrainBatInvMotDiff ptn(
    SOC_start = 0.85,
    diff_driveSideTorqueSign = 1,
    diff_T_preload = 25,
    diff_lockFractionAccel = 0.40,
    diff_lockFractionDecel = 0.20,
    diff_T_capacity_max = 250,
    diff_clutchEffectiveRadius = 1.0,
    diff_kineticFrictionRatio = 0.80,
    diff_w_transition = 1.5,
    diff_c_viscous = 0) annotation(
      Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed mountFixture annotation(
    Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Mechanics.Rotational.Components.Inertia leftWheelInertia(
    J = 1.2,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
      Placement(transformation(origin = {-60, -8}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia rightWheelInertia(
    J = 1.0,
    phi(start = 0, fixed = true),
    w(start = 0, fixed = true)) annotation(
      Placement(transformation(origin = {60, -8}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Vehicle.Electronics.ElectronicsAssembly elc(
    tau_max = 220,
    w_eps = 1.0,
    motorSpeedSign = 1) annotation(
      Placement(transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}})));

equation
  elc.cmd_torque_motor =
    60*(1 + Modelica.Math.tanh((time - 0.275)/0.03))
    - 80*(1 + Modelica.Math.tanh((time - 1.175)/0.03))
    + 20*(1 + Modelica.Math.tanh((time - 1.875)/0.03));
  elc.cmd_regen_limit = 80;
  elc.cmd_inverter_enable = true;
  elc.sens_motor_speed = ptn.motorSpeed;

  assert(ptn.SOE >= -1e-6 and ptn.SOE <= 1 + 1e-6,
    "PowertrainBatInvMotDiff SOE left normalized bounds");
  assert(abs(ptn.diffLockTorque) <= ptn.diffLockCapacity + 1e-6,
    "PowertrainBatInvMotDiff differential lock exceeded its dynamic capacity");
  assert(abs(elc.tau_cmd_limited) <= elc.tau_max + 1e-6,
    "PowertrainBatInvMotDiff VCU torque limit was exceeded");

  connect(elc.P_motor, ptn.P_elec) annotation(
    Line(points = {{-48, -40}, {-26, -40}, {-26, -24}, {-12, -24}}, color = {0, 0, 127}));
  connect(elc.p, ptn.hv_p) annotation(
    Line(points = {{-70, -36}, {-80, -36}, {-80, 8}, {-20, 8}}, color = {0, 0, 255}));
  connect(elc.n, ptn.hv_n) annotation(
    Line(points = {{-70, -44}, {-86, -44}, {-86, 2}, {-20, 2}}, color = {0, 0, 255}));
  connect(mountFixture.frame_b, ptn.mountFrame) annotation(
    Line(points = {{0, 40}, {0, 20}}, color = {95, 95, 95}));
  connect(ptn.leftFlange, leftWheelInertia.flange_a) annotation(
    Line(points = {{-20, -8}, {-70, -8}}));
  connect(ptn.rightFlange, rightWheelInertia.flange_a) annotation(
    Line(points = {{20, -8}, {50, -8}}));

  annotation(
    experiment(StartTime = 0, StopTime = 2.0, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end TestPowertrainBatInvMotDiff;
