within BobLibTest.Controllers;
model TestVCU

  BobLib.Controllers.VCU vcu(pVCU(tau_max = 200, w_eps = 0.1)) annotation(
    Placement(transformation(origin = {100, 40}, extent = {{-20, -20}, {20, 20}})));
  BobLib.Utilities.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus annotation(
    Placement(transformation(origin = {-20, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  BobLib.Chassis.Interfaces.ChassisBus chassisBus annotation(
    Placement(transformation(origin = {-20, -40}, extent = {{-10, -10}, {10, 10}})));
  BobLib.ElectricDrives.Interfaces.ElectricMotorBus electricMotorBus annotation(
    Placement(transformation(origin = {-20, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant acceleratorPedal(k = 0.15) annotation(
    Placement(transformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant brakePedal(k = 0) annotation(
    Placement(transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnable(k = true) annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant vehicleSpeed(k = 0) annotation(
    Placement(transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant motorSpeed(k = 100) annotation(
    Placement(transformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{40, 0}, {100, 0}, {100, 20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.driverBus, driverBus) annotation(
    Line(points = {{40, 0}, {40, 100}, {-20, 100}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{40, 0}, {40, -40}, {-20, -40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.electricMotorBus, electricMotorBus) annotation(
    Line(points = {{40, 0}, {40, -80}, {-20, -80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(acceleratorPedal.y, driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-89, 80}, {-20, 80}, {-20, 100}}, color = {0, 0, 127}));
  connect(brakePedal.y, driverBus.brakePedalPosition) annotation(
    Line(points = {{-89, 40}, {-20, 40}, {-20, 100}}, color = {0, 0, 127}));
  connect(inverterEnable.y, driverBus.inverterEnable) annotation(
    Line(points = {{-89, 0}, {-20, 0}, {-20, 100}}, color = {255, 0, 255}));
  connect(vehicleSpeed.y, chassisBus.vehicleSpeed) annotation(
    Line(points = {{-89, -40}, {-20, -40}}, color = {0, 0, 127}));
  connect(motorSpeed.y, electricMotorBus.speed) annotation(
    Line(points = {{-89, -80}, {-20, -80}}, color = {0, 0, 127}));
  assert(vcu.vcu_active, "VCU enable path changed");

  annotation(experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-6, Interval = 0.01),
    Diagram(coordinateSystem(extent = {{-140, -100}, {140, 120}}, preserveAspectRatio = true)));
end TestVCU;
