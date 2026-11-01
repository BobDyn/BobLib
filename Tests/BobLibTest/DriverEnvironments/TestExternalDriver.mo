within BobLibTest.DriverEnvironments;
model TestExternalDriver

  parameter Real steeringNormalized = 0.5 "Normalized steering input applied by the driver-environment fixture" annotation(Dialog(group = "Operating settings"));
  parameter Real pedal = 0.4 "Normalized brake demand applied by the driver-environment fixture" annotation(Dialog(group = "Operating settings"));

  VehicleInterfaces.Drivers.MinimalDriver driver(
    steeringAngle(k = steeringNormalized), initialAccelRequest = pedal,
    initialBrakeRequest = pedal, manualGearRequest = 2)
    annotation(Placement(transformation(origin = {-70, 30}, extent = {{-20, -20}, {20, 20}})));

  BobLib.DriverEnvironments.ExternalDriver dut(steeringRange = 4)
    annotation(Placement(transformation(origin = {50, 30}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.Constant speed(k = 12)
    annotation(Placement(transformation(origin = {-20, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant motorSpeed(k = 100)
    annotation(Placement(transformation(origin = {-20, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput speedFeedback annotation(
    Placement(transformation(origin = {-120, -20}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealOutput motorFeedback annotation(
    Placement(transformation(origin = {-120, -70}, extent = {{-8, -8}, {8, 8}}, rotation = 180)));

equation
  connect(driver.driverInterface, dut.driverInterface) annotation(
    Line(points = {{-50, 30}, {30, 30}}, color = {85, 85, 255}, thickness = 0.5));
  connect(speed.y, dut.controlBus.chassisBus.vehicleSpeed) annotation(
    Line(points = {{-9, -40}, {90, -40}, {90, 42}, {70, 42}}, color = {0, 0, 127}));
  connect(motorSpeed.y, dut.controlBus.electricMotorBus.speed) annotation(
    Line(points = {{-9, -70}, {100, -70}, {100, 42}, {70, 42}}, color = {0, 0, 127}));
  connect(driver.driverInterface.vehicleSpeed, speedFeedback) annotation(
    Line(points = {{-50, 30}, {-40, 30}, {-40, -20}, {-120, -20}}, color = {0, 0, 127}));
  connect(driver.driverInterface.motorSpeed, motorFeedback) annotation(
    Line(points = {{-50, 30}, {-35, 30}, {-35, -90}, {-100, -90}, {-100, -70}, {-120, -70}}, color = {0, 0, 127}));
  assert(abs(dut.steeringWheel.phi-4*(steeringNormalized-0.5))<1e-10,
    "VI steering center/range conversion");
  assert(abs(dut.actuation.acceleratorPedalCommand-pedal)<1e-12 and
    abs(dut.actuation.brakePedalCommand-pedal)<1e-12, "VI pedal conversion");
  assert(abs(speedFeedback-12)<1e-10 and abs(motorFeedback-100)<1e-10, "VI driver feedback");
  assert(dut.requestedGear.y==2, "VI requested gear forwarding");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-120, -120}, {120, 80}}, preserveAspectRatio = true)));
end TestExternalDriver;
