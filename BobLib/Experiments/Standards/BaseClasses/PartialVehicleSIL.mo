within BobLib.Experiments.Standards.BaseClasses;

partial model PartialVehicleSIL "Car-only actuator and sensor boundary for an external VCU"

  import Modelica.Units.SI;

  extends PartialElectricVehicle(
    initialVel = 0,
    redeclare replaceable BobLib.DriverEnvironments.CommandedDriveByWire driverEnvironment(
      exactSteering = false, steeringFilterFrequency = steeringFilterFrequency)
      constrainedby BobLib.DriverEnvironments.Interfaces.Commanded);

  parameter SI.Frequency steeringFilterFrequency = 50
    "Position filter cutoff for sampled steering-wheel commands" annotation(Dialog(group = "Operating settings"));

  Modelica.Blocks.Interfaces.RealInput motorTorqueCommand(unit = "N.m", start = 0)
    "Signed motor-axis torque request before physical drive losses and limits; positive drives forward" annotation(
    Placement(transformation(origin = {-200, 110}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-100, 75}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.BooleanInput inverterEnable(start = false)
    "Allow the requested drive power; false requests zero power with the motor's normal torque decay" annotation(
    Placement(transformation(origin = {-200, 85}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-100, 25}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput brakeCommand(unit = "1", min = 0, max = 1, start = 0)
    "Mechanical brake fraction of pVehicle.brakes.maxTorque; independent of regen" annotation(
    Placement(transformation(origin = {180, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 180),
      iconTransformation(origin = {-100, -25}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(unit = "rad", start = 0)
    "Steering-wheel angle, not road-wheel angle" annotation(
    Placement(transformation(origin = {180, 85}, extent = {{-10, -10}, {10, 10}}, rotation = 180),
      iconTransformation(origin = {-100, -75}, extent = {{-10, -10}, {10, 10}})));

  output SI.Velocity vehicleSpeed "True nonnegative speed magnitude; use velX for signed forward speed";
  output Real motorSpeed(unit = "rev/min") "Signed motor shaft speed in revolutions per minute";
  output SI.AngularVelocity wheelSpeedFL "Front-left wheel angular velocity";
  output SI.AngularVelocity wheelSpeedFR "Front-right wheel angular velocity";
  output SI.AngularVelocity wheelSpeedRL "Rear-left wheel angular velocity";
  output SI.AngularVelocity wheelSpeedRR "Rear-right wheel angular velocity";
  output SI.Voltage dcVoltage "Battery-side inverter DC bus voltage";
  output SI.Current dcCurrent "Battery-side inverter DC current; positive discharge, negative charge";
  output SI.Power requestedPower "Drive power request before inverter physical limits";
  output SI.Power motorMechanicalPower "Actual motor mechanical power, positive motoring";
  output SI.Torque motorTorqueTarget "Physical motor torque target after losses and limits, before actuator lag";

protected
  discrete Real shaftDirection(start = 1, fixed = true)
    "Same direction latch as PowerLimitedMotor for power conversion near zero speed";
  SI.AngularVelocity regularizedSpeed "Motor direction and regularized magnitude used by the physical drive";
  SI.AngularVelocity shaftSpeed "Signed speed from the motor subsystem bus";

initial equation
  assert(steeringFilterFrequency > 0, "VehicleSIL: steeringFilterFrequency must be positive");

equation
  assert(noEvent(abs(motorTorqueCommand) < Modelica.Constants.inf and
    abs(steeringAngleCommand) < Modelica.Constants.inf and brakeCommand >= 0 and brakeCommand <= 1),
    "BOBLIB_INVALID_SIL_INPUT: commands must be finite and brakeCommand must be in [0,1]");

  // Adapt the actuator request to the existing power-controlled drive. There is
  // no pedal map, VCU power cap, slip controller, launch ramp or brake blending.
  when initial() then
    shaftDirection = if abs(shaftSpeed) > pVehicle.motor.w_eps then
      (if shaftSpeed > 0 then 1 else -1) else(if pVehicle.motor.reverseLaunch then -1 else 1);
  elsewhen shaftSpeed > pVehicle.motor.w_eps then
    shaftDirection = 1;
  elsewhen shaftSpeed < -pVehicle.motor.w_eps then
    shaftDirection = -1;
  end when;
  shaftSpeed = controlBus.electricMotorBus.speed;
  regularizedSpeed = shaftDirection*sqrt(shaftSpeed*shaftSpeed + pVehicle.motor.w_eps*pVehicle.motor.w_eps);
  requestedPower = if inverterEnable then motorTorqueCommand*regularizedSpeed else 0;
  controlBus.electricMotorControlBus.powerRequest = requestedPower;
  controlBus.brakesControlBus.mechanicalBrakeTorqueRequest = brakeCommand*pVehicle.brakes.maxTorque;
  driverEnvironment.acceleratorPedalCommand = 0;
  connect(brakeCommand, driverEnvironment.brakePedalCommand) annotation(
    Line(points = {{180, 110}, {0, 110}, {0, 64}, {18, 64}}, color = {0, 0, 127}));
  connect(steeringAngleCommand, driverEnvironment.steeringAngleCommand) annotation(
    Line(points = {{180, 85}, {10, 85}, {10, 76}, {18, 76}}, color = {0, 0, 127}));

  vehicleSpeed = chassis.vehicleSpeed;
  motorSpeed = shaftSpeed*30/Modelica.Constants.pi;
  wheelSpeedFL = controlBus.brakesBus.wheelSpeed_1;
  wheelSpeedFR = controlBus.brakesBus.wheelSpeed_2;
  wheelSpeedRL = controlBus.brakesBus.wheelSpeed_3;
  wheelSpeedRR = controlBus.brakesBus.wheelSpeed_4;
  dcVoltage = controlBus.batteryBus.voltage;
  dcCurrent = controlBus.batteryBus.current;
  motorMechanicalPower = controlBus.electricMotorBus.mechanicalPower;
  motorTorqueTarget = controlBus.electricMotorBus.torqueCommand;

  annotation(Diagram(coordinateSystem(extent = {{-200, -180}, {180, 120}})),
    Documentation(info = "<html><p>Car-only boundary for an external firmware VCU. Battery, inverter, motor, driveline, brakes, chassis and environment remain physical BobLib subsystems. No BobLib controller is instantiated. Commands are held by the FMI host between exchanges; the internal solver resolves faster physical dynamics.</p><p>Signed torque is converted to power using the motor's direction latch and smooth near-zero speed magnitude. Existing inverter efficiency, free-running motor losses, torque/power limits and torque dynamics still apply, so requested and delivered torque differ. This is the existing averaged drive approximation, not an ideal torque source. Negative torque at forward speed requests regeneration; the external VCU owns low-speed regen gating.</p><p>Mechanical braking is independent of torque and inverter enable. The brake input is a normalized actuator command, not hydraulic pressure. Pedals, ready-to-drive, shutdown and CAN freshness belong to the external harness/VCU. Wheel speeds and DC measurements are ideal model measurements; noise, delays, rates and CAN encoding belong to the harness. Inherited velX, accX, accY and tire loads are truth signals.</p></html>"));
end PartialVehicleSIL;
