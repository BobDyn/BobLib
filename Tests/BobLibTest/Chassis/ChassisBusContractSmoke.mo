within BobLibTest.Chassis;
model ChassisBusContractSmoke "Chassis measurements agree with the published bus"

  extends BobLib.Experiments.Standards.VehicleSim;

  output Modelica.Units.SI.Force measuredLoads[4] = {
    controlBus.chassisBus.Fz_1, controlBus.chassisBus.Fz_2,
    controlBus.chassisBus.Fz_3, controlBus.chassisBus.Fz_4}
    "Corner loads observed on the chassis bus";
  output Modelica.Units.SI.Length measuredLengths[4] = {
    controlBus.chassisBus.frontLeftSpringLength,
    controlBus.chassisBus.frontRightSpringLength,
    controlBus.chassisBus.rearLeftSpringLength,
    controlBus.chassisBus.rearRightSpringLength}
    "Spring lengths observed on the chassis bus";
  output Modelica.Units.SI.Velocity signedSpeed = controlBus.chassisBus.longitudinalVelocity
    "Signed longitudinal velocity observed on the chassis bus";
  output Modelica.Units.SI.Velocity speedMagnitude = controlBus.chassisBus.vehicleSpeed
    "Speed magnitude observed on the chassis bus";

equation
  assert(
    max(abs(measuredLoads - {chassis.Fz_1, chassis.Fz_2, chassis.Fz_3, chassis.Fz_4})) < 1e-8,
    "Chassis bus load mapping differs from chassis outputs");
  assert(
    max(abs(measuredLengths - {chassis.frontLeftSpringLength, chassis.frontRightSpringLength, chassis.rearLeftSpringLength, chassis.rearRightSpringLength})) < 1e-10,
    "Chassis bus spring mapping differs from chassis outputs");
  assert(
    abs(signedSpeed - chassis.bodyVelocity[1]) < 1e-10 and abs(speedMagnitude - chassis.vehicleSpeed) < 1e-10,
    "Chassis bus speed mapping differs from chassis outputs");
  annotation(experiment(StopTime = 0.01));
end ChassisBusContractSmoke;
