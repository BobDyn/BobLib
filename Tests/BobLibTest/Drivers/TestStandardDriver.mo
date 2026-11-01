within BobLibTest.Drivers;
model TestStandardDriver

  BobLib.Drivers.StandardDriver driver(targetAy = 1, driveTorqueCapacity = 200)
    annotation(Placement(transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}})));

  BobLib.DriverEnvironments.ExternalDriver environment(includeManeuverMeasurements = true,
    steeringCenter = driver.steeringCenter, steeringRange = driver.steeringRange)
    annotation(Placement(transformation(origin = {100, 80}, extent = {{-20, -20}, {20, 20}})));
  BobLib.Chassis.Interfaces.ChassisBus chassisBus annotation(Placement(transformation(origin = {170, -20}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Blocks.Sources.Constant vehicleSpeed(k = 0) annotation(Placement(transformation(origin = {-150, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant lateralAcceleration(k = 0) annotation(Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant motorSpeed(k = 100) annotation(Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant normalLoad_1(k = 1000) annotation(Placement(transformation(origin = {-150, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant normalLoad_2(k = 1000) annotation(Placement(transformation(origin = {-50, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant normalLoad_3(k = 1000) annotation(Placement(transformation(origin = {50, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant normalLoad_4(k = 1000) annotation(Placement(transformation(origin = {150, -80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(driver.driverInterface, environment.driverInterface) annotation(Line(points = {{-80, 80}, {80, 80}}, color = {85, 85, 255}, thickness = 0.5));
  connect(chassisBus, environment.controlBus.chassisBus) annotation(Line(points = {{170, -20}, {190, -20}, {190, 92}, {120, 92}}, color = {255, 204, 51}));
  connect(vehicleSpeed.y, chassisBus.vehicleSpeed) annotation(Line(points = {{-139, 0}, {-130, 0}, {-130, 30}, {170, 30}, {170, -20}}, color = {0, 0, 127}));
  connect(lateralAcceleration.y, chassisBus.bodyAcceleration_2) annotation(Line(points = {{-39, 0}, {-30, 0}, {-30, 20}, {174, 20}, {174, -20}, {170, -20}}, color = {0, 0, 127}));
  connect(motorSpeed.y, environment.controlBus.electricMotorBus.speed) annotation(Line(points = {{61, 50}, {180, 50}, {180, 92}, {120, 92}}, color = {0, 0, 127}));
  connect(normalLoad_1.y, chassisBus.Fz_1) annotation(Line(points = {{-139, -80}, {-134, -80}, {-134, -40}, {160, -40}, {160, -20}, {170, -20}}, color = {0, 0, 127}));
  connect(normalLoad_2.y, chassisBus.Fz_2) annotation(Line(points = {{-39, -80}, {-34, -80}, {-34, -48}, {163, -48}, {163, -20}, {170, -20}}, color = {0, 0, 127}));
  connect(normalLoad_3.y, chassisBus.Fz_3) annotation(Line(points = {{61, -80}, {66, -80}, {66, -56}, {166, -56}, {166, -20}, {170, -20}}, color = {0, 0, 127}));
  connect(normalLoad_4.y, chassisBus.Fz_4) annotation(Line(points = {{161, -80}, {166, -80}, {166, -64}, {169, -64}, {169, -20}, {170, -20}}, color = {0, 0, 127}));
  assert(driver.minTireNormalLoad>0, "Driver receives chassis loads through VI interface");
  assert(abs(environment.actuation.steeringAngleCommand-driver.frSteerCmd)<1e-10, "Steering conversion round trip");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-200, -120}, {220, 140}}, preserveAspectRatio = true)));
end TestStandardDriver;
