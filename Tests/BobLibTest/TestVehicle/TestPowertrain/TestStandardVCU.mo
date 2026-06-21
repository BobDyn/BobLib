within BobLibTest.TestVehicle.TestPowertrain;

model TestStandardVCU

  BobLib.Controllers.StandardVCU vcu(
    useMode = BobLib.Controllers.StandardVCU.MODE_OPEN_LOOP_RAMP,
    targetAy = 1,
    tau_max = 200,
    w_eps = 0.1) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

  VehicleInterfaces.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
  VehicleInterfaces.Interfaces.DriverBus driverBus annotation(
    Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation(
    Placement(transformation(extent = {{-80, 10}, {-60, 30}})));
  VehicleInterfaces.Interfaces.BatteryBus batteryBus annotation(
    Placement(transformation(extent = {{-80, -30}, {-60, -10}})));
  VehicleInterfaces.Interfaces.ElectricMotorBus electricMotorBus annotation(
    Placement(transformation(extent = {{-80, -50}, {-60, -30}})));

  Modelica.Blocks.Sources.Constant steeringAngle(k = 0) annotation(
    Placement(transformation(extent = {{-120, 44}, {-100, 64}})));

  Modelica.Blocks.Sources.Constant acceleratorPedal(k = 0) annotation(
    Placement(transformation(extent = {{-120, 34}, {-100, 54}})));

  Modelica.Blocks.Sources.Constant brakePedal(k = 0) annotation(
    Placement(transformation(extent = {{-120, 24}, {-100, 44}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnable(k = true) annotation(
    Placement(transformation(extent = {{-120, 14}, {-100, 34}})));

  Modelica.Blocks.Sources.Constant vehicleSpeed(k = 0) annotation(
    Placement(transformation(extent = {{-120, -56}, {-100, -36}})));

  Modelica.Blocks.Sources.Constant motorSpeed(k = 100) annotation(
    Placement(transformation(extent = {{-120, -76}, {-100, -56}})));

  Modelica.Blocks.Sources.Constant hvVoltage(k = 400) annotation(
    Placement(transformation(extent = {{-120, -96}, {-100, -76}})));

  Modelica.Blocks.Sources.Constant hvCurrent(k = 20) annotation(
    Placement(transformation(extent = {{-120, -116}, {-100, -96}})));

  Modelica.Blocks.Sources.Constant normalLoad_1(k = 1000) annotation(
    Placement(transformation(extent = {{-120, 4}, {-100, 24}})));

  Modelica.Blocks.Sources.Constant normalLoad_2(k = 1000) annotation(
    Placement(transformation(extent = {{-120, -6}, {-100, 14}})));

  Modelica.Blocks.Sources.Constant normalLoad_3(k = 1000) annotation(
    Placement(transformation(extent = {{-120, -16}, {-100, 4}})));

  Modelica.Blocks.Sources.Constant normalLoad_4(k = 1000) annotation(
    Placement(transformation(extent = {{-120, -26}, {-100, -6}})));

  Modelica.Blocks.Sources.Constant lateralAcceleration(k = 0) annotation(
    Placement(transformation(extent = {{-120, -36}, {-100, -16}})));

equation
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{-70, 0}, {0, 0}, {0, -10}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.driverBus, driverBus) annotation(
    Line(points = {{-70, 0}, {-70, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{-70, 0}, {-70, 20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.batteryBus, batteryBus) annotation(
    Line(points = {{-70, 0}, {-70, -20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorBus, electricMotorBus) annotation(
    Line(points = {{-70, 0}, {-70, -40}}, color = {255, 204, 51}, thickness = 0.5));

  connect(steeringAngle.y, driverBus.steeringWheelAngle) annotation(
    Line(points = {{-99, 54}, {-70, 54}, {-70, 40}}, color = {0, 0, 127}));
  connect(acceleratorPedal.y, driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-99, 44}, {-74, 44}, {-74, 40}, {-70, 40}}, color = {0, 0, 127}));
  connect(brakePedal.y, driverBus.brakePedalPosition) annotation(
    Line(points = {{-99, 34}, {-70, 34}, {-70, 40}}, color = {0, 0, 127}));
  connect(inverterEnable.y, driverBus.inverterEnable) annotation(
    Line(points = {{-99, 24}, {-76, 24}, {-76, 40}, {-70, 40}}, color = {255, 0, 255}));
  connect(vehicleSpeed.y, chassisBus.vehicleSpeed) annotation(
    Line(points = {{-99, -46}, {-70, -46}, {-70, 20}}, color = {0, 0, 127}));
  connect(normalLoad_1.y, chassisBus.Fz_1) annotation(
    Line(points = {{-99, 14}, {-70, 14}, {-70, 20}}, color = {0, 0, 127}));
  connect(normalLoad_2.y, chassisBus.Fz_2) annotation(
    Line(points = {{-99, 4}, {-72, 4}, {-72, 20}, {-70, 20}}, color = {0, 0, 127}));
  connect(normalLoad_3.y, chassisBus.Fz_3) annotation(
    Line(points = {{-99, -6}, {-74, -6}, {-74, 20}, {-70, 20}}, color = {0, 0, 127}));
  connect(normalLoad_4.y, chassisBus.Fz_4) annotation(
    Line(points = {{-99, -16}, {-76, -16}, {-76, 20}, {-70, 20}}, color = {0, 0, 127}));
  connect(lateralAcceleration.y, chassisBus.bodyAcceleration_2) annotation(
    Line(points = {{-99, -26}, {-78, -26}, {-78, 20}, {-70, 20}}, color = {0, 0, 127}));
  connect(motorSpeed.y, electricMotorBus.speed) annotation(
    Line(points = {{-99, -66}, {-70, -66}, {-70, -40}}, color = {0, 0, 127}));
  connect(hvVoltage.y, batteryBus.voltage) annotation(
    Line(points = {{-99, -86}, {-70, -86}, {-70, -20}}, color = {0, 0, 127}));
  connect(hvCurrent.y, batteryBus.current) annotation(
    Line(points = {{-99, -106}, {-66, -106}, {-66, -20}, {-70, -20}}, color = {0, 0, 127}));

  assert(vcu.vcu_active, "StandardVCU inherited enable path changed");
  assert(vcu.minTireNormalLoad > 0, "StandardVCU chassisBus subscription changed");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestStandardVCU;
