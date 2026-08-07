within BobLibTest.TestVehicle.TestPowertrain;

model TestPowerLimitedMotor

  import SI = Modelica.Units.SI;

  BobLib.ElectricDrives.Internal.PowerLimitedMotor motoringForward(
    enablePeakTimer = false);

  BobLib.ElectricDrives.Internal.PowerLimitedMotor motoringReverse(
    enablePeakTimer = false);

  BobLib.ElectricDrives.Internal.PowerLimitedMotor regenForward(
    enablePeakTimer = false);

  BobLib.ElectricDrives.Internal.PowerLimitedMotor regenReverse(
    enablePeakTimer = false);

  BobLib.ElectricDrives.Internal.PowerLimitedMotor forwardLaunch(
    enablePeakTimer = false);

  BobLib.ElectricDrives.Internal.PowerLimitedMotor reverseLaunch(
    enablePeakTimer = false,
    reverseLaunch = true);

  Modelica.Blocks.Sources.Constant positivePower(k = 10000);

  Modelica.Blocks.Sources.Constant negativePower(k = -10000);

  Modelica.Blocks.Sources.Constant positiveSpeed(k = 100);

  Modelica.Blocks.Sources.Constant negativeSpeed(k = -100);

  Modelica.Blocks.Sources.Constant zeroSpeed(k = 0);

  Modelica.Mechanics.Rotational.Sources.Speed positiveSpeedMotoring(
    exact = true);

  Modelica.Mechanics.Rotational.Sources.Speed negativeSpeedMotoring(
    exact = true);

  Modelica.Mechanics.Rotational.Sources.Speed positiveSpeedRegen(
    exact = true);

  Modelica.Mechanics.Rotational.Sources.Speed negativeSpeedRegen(
    exact = true);

  Modelica.Mechanics.Rotational.Sources.Speed standstill(
    exact = true);

  Modelica.Mechanics.Rotational.Sources.Speed reverseStandstill(
    exact = true);

  output SI.Power forwardMotoringPower = motoringForward.P_mech;
  output SI.Power reverseMotoringPower = motoringReverse.P_mech;
  output SI.Power forwardRegenPower = regenForward.P_mech;
  output SI.Power reverseRegenPower = regenReverse.P_mech;
  output SI.Torque forwardMotoringTorque = motoringForward.tau_cmd;
  output SI.Torque reverseMotoringTorque = motoringReverse.tau_cmd;
  output SI.Torque forwardRegenTorque = regenForward.tau_cmd;
  output SI.Torque reverseRegenTorque = regenReverse.tau_cmd;
  output SI.Torque forwardLaunchTorque = forwardLaunch.tau_cmd;
  output SI.Torque reverseLaunchTorque = reverseLaunch.tau_cmd;

equation
  connect(positivePower.y, motoringForward.P_elec);
  connect(positivePower.y, motoringReverse.P_elec);
  connect(negativePower.y, regenForward.P_elec);
  connect(negativePower.y, regenReverse.P_elec);
  connect(positivePower.y, forwardLaunch.P_elec);
  connect(positivePower.y, reverseLaunch.P_elec);

  connect(positiveSpeed.y, positiveSpeedMotoring.w_ref);
  connect(negativeSpeed.y, negativeSpeedMotoring.w_ref);
  connect(positiveSpeed.y, positiveSpeedRegen.w_ref);
  connect(negativeSpeed.y, negativeSpeedRegen.w_ref);
  connect(zeroSpeed.y, standstill.w_ref);
  connect(zeroSpeed.y, reverseStandstill.w_ref);

  connect(positiveSpeedMotoring.flange, motoringForward.shaft);
  connect(negativeSpeedMotoring.flange, motoringReverse.shaft);
  connect(positiveSpeedRegen.flange, regenForward.shaft);
  connect(negativeSpeedRegen.flange, regenReverse.shaft);
  connect(standstill.flange, forwardLaunch.shaft);
  connect(reverseStandstill.flange, reverseLaunch.shaft);

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Interval = 0.001, Tolerance = 1e-8),
    Documentation(info = "<html>
<p>
Regression fixture for motor power/torque sign behavior at positive speed,
negative speed, and standstill. It covers motoring, regeneration, and the
configurable forward/reverse launch convention.
</p>
</html>"));
end TestPowerLimitedMotor;
