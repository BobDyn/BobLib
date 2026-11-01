within BobLibTest.DriverEnvironments;

model TestAutomaticDriveByWire

  BobLib.DriverEnvironments.AutomaticDriveByWire dut(
    steeringAngleCommand = 0.1*time,
    acceleratorPedalCommand = 0.4,
    brakePedalCommand = 0.2,
    manualGearRequest = 2,
    gearboxModeCommand = 1,
    ignitionCommand = 1) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput accelerator annotation(Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.IntegerOutput requestedGear annotation(Placement(transformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(dut.controlBus.driverBus.requestedGear, requestedGear) annotation(Line(points = {{10, 6}, {20, 6}, {20, 30}, {100, 30}}, color = {255, 127, 0}));
  assert(requestedGear == 2, "Manual gear request publication");
  connect(dut.controlBus.driverBus.acceleratorPedalPosition, accelerator) annotation(Line(points = {{10, 6}, {13, 6}, {13, 0}, {100, 0}}, color = {0, 0, 127}));
  assert(abs(accelerator - 0.4) < 1e-12, "Accelerator bus publication");
  assert(abs(dut.steeringWheel.phi - 0.1*time) < 1e-10, "Steering command tracking");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-40, -40}, {100, 40}}, preserveAspectRatio = true)));
end TestAutomaticDriveByWire;
