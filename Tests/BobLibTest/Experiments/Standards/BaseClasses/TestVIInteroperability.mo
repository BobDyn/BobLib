within BobLibTest.Experiments.Standards.BaseClasses;
model TestVIInteroperability "Generic boundary accepts stock VI subsystem replacements"

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicle(
    redeclare VehicleInterfaces.Chassis.MinimalChassis chassis,
    redeclare VehicleInterfaces.DriverEnvironments.DriveByWireAutomatic driverEnvironment,
    redeclare VehicleInterfaces.Brakes.NoBrakes brakes,
    redeclare VehicleInterfaces.Drivelines.NoDriveline driveline);
  VehicleInterfaces.Transmissions.NoTransmissions transmission annotation(
    Placement(transformation(origin = {-40, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Fixed shaftReference annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutput speed annotation(
    Placement(transformation(origin = {180, 90}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(shaftReference.flange, transmission.engineFlange.flange) annotation(
    Line(points = {{-90, -50}, {-50, -50}}, color = {0, 0, 0}));
  connect(transmission.drivelineFlange, driveline.transmissionFlange) annotation(
    Line(points = {{-30, -50}, {10, -50}}, color = {135, 135, 135}));
  connect(controlBus, transmission.controlBus) annotation(
    Line(points = {{-200, 30}, {-60, 30}, {-60, -44}, {-50, -44}}, color = {255, 204, 51}));
  connect(controlBus.chassisBus.longitudinalVelocity, speed) annotation(
    Line(points = {{-200, 30}, {-180, 30}, {-180, 90}, {180, 90}}, color = {0, 0, 127}));
  connect(driveline.wheelHub_1, chassis.wheelHub_1) annotation(Line(points = {{16, -65}, {16, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_1, chassis.wheelHub_1) annotation(Line(points = {{146, -64}, {146, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_2, chassis.wheelHub_2) annotation(Line(points = {{16, -35}, {16, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(Line(points = {{146, -34}, {146, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(Line(points = {{34, -65}, {34, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_3, chassis.wheelHub_3) annotation(Line(points = {{164, -64}, {164, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(Line(points = {{34, -35}, {34, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_4, chassis.wheelHub_4) annotation(Line(points = {{164, -34}, {164, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  assert(abs(speed)<1e-10, "Null powertrain/brakes must leave the stationary chassis at rest");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6));
end TestVIInteroperability;
