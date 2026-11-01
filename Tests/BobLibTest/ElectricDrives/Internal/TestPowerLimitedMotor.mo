within BobLibTest.ElectricDrives.Internal;

model TestPowerLimitedMotor

  import Modelica.Units.SI;

  BobLib.ElectricDrives.Internal.PowerLimitedMotor motoringForward(pMotor(enablePeakTimer = false)) annotation(Placement(transformation(origin = {-100, 110}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor motoringReverse(pMotor(enablePeakTimer = false)) annotation(Placement(transformation(origin = {-100, 0}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor regenForward(pMotor(enablePeakTimer = false)) annotation(Placement(transformation(origin = {200, 110}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor regenReverse(pMotor(enablePeakTimer = false)) annotation(Placement(transformation(origin = {200, 0}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor forwardLaunch(pMotor(enablePeakTimer = false)) annotation(Placement(transformation(origin = {-100, -110}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor reverseLaunch(
    pMotor(
      enablePeakTimer = false,
      reverseLaunch = true)) annotation(Placement(transformation(origin = {200, -110}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant positivePower(k = 10000) annotation(Placement(transformation(origin = {-160, 200}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant negativePower(k = -10000) annotation(Placement(transformation(origin = {160, 200}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant positiveSpeed(k = 100) annotation(Placement(transformation(origin = {-320, 140}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant negativeSpeed(k = -100) annotation(Placement(transformation(origin = {-320, 40}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant zeroSpeed(k = 0) annotation(Placement(transformation(origin = {-320, -60}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed positiveSpeedMotoring(
    exact = true) annotation(Placement(transformation(origin = {-190, 110}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed negativeSpeedMotoring(
    exact = true) annotation(Placement(transformation(origin = {-190, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed positiveSpeedRegen(
    exact = true) annotation(Placement(transformation(origin = {110, 110}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed negativeSpeedRegen(
    exact = true) annotation(Placement(transformation(origin = {110, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed standstill(
    exact = true) annotation(Placement(transformation(origin = {-190, -110}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed reverseStandstill(
    exact = true) annotation(Placement(transformation(origin = {110, -110}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

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
  connect(positivePower.y, motoringForward.P_elec) annotation(Line(points = {{-143.5, 200}, {-142, 200}, {-142, 110}}, color = {0, 0, 127}));
  connect(positivePower.y, motoringReverse.P_elec) annotation(Line(points = {{-143.5, 200}, {-138, 200}, {-138, 0}, {-142, 0}}, color = {0, 0, 127}));
  connect(negativePower.y, regenForward.P_elec) annotation(Line(points = {{176.5, 200}, {178, 200}, {178, 151}, {162, 151}, {162, 110}, {158, 110}}, color = {0, 0, 127}));
  connect(negativePower.y, regenReverse.P_elec) annotation(Line(points = {{176.5, 200}, {176.5, 161}, {158, 161}, {158, 0}}, color = {0, 0, 127}));
  connect(positivePower.y, forwardLaunch.P_elec) annotation(Line(points = {{-143.5, 200}, {-143.5, -110}, {-142, -110}}, color = {0, 0, 127}));
  connect(positivePower.y, reverseLaunch.P_elec) annotation(Line(points = {{-143.5, 200}, {-142, 200}, {-142, 201}, {142, 201}, {142, -110}, {158, -110}}, color = {0, 0, 127}));

  connect(positiveSpeed.y, positiveSpeedMotoring.w_ref) annotation(Line(points = {{-303.5, 140}, {-302, 140}, {-302, 110}, {-208, 110}}, color = {0, 0, 127}));
  connect(negativeSpeed.y, negativeSpeedMotoring.w_ref) annotation(Line(points = {{-303.5, 40}, {-302, 40}, {-302, 0}, {-208, 0}}, color = {0, 0, 127}));
  connect(positiveSpeed.y, positiveSpeedRegen.w_ref) annotation(Line(points = {{-303.5, 140}, {-302, 140}, {-302, 151}, {92, 151}, {92, 110}}, color = {0, 0, 127}));
  connect(negativeSpeed.y, negativeSpeedRegen.w_ref) annotation(Line(points = {{-303.5, 40}, {92, 40}, {92, 0}}, color = {0, 0, 127}));
  connect(zeroSpeed.y, standstill.w_ref) annotation(Line(points = {{-303.5, -60}, {-302, -60}, {-302, -110}, {-208, -110}}, color = {0, 0, 127}));
  connect(zeroSpeed.y, reverseStandstill.w_ref) annotation(Line(points = {{-303.5, -60}, {92, -60}, {92, -110}}, color = {0, 0, 127}));

  connect(positiveSpeedMotoring.flange, motoringForward.shaft) annotation(Line(points = {{-175, 110}, {-172, 110}, {-172, 71}, {-62, 71}, {-62, 110}, {-65, 110}}, color = {0, 0, 0}));
  connect(negativeSpeedMotoring.flange, motoringReverse.shaft) annotation(Line(points = {{-175, 0}, {-172, 0}, {-172, -39}, {-62, -39}, {-62, 0}, {-65, 0}}, color = {0, 0, 0}));
  connect(positiveSpeedRegen.flange, regenForward.shaft) annotation(Line(points = {{125, 110}, {128, 110}, {128, 71}, {238, 71}, {238, 110}, {235, 110}}, color = {0, 0, 0}));
  connect(negativeSpeedRegen.flange, regenReverse.shaft) annotation(Line(points = {{125, 0}, {128, 0}, {128, -39}, {238, -39}, {238, 0}, {235, 0}}, color = {0, 0, 0}));
  connect(standstill.flange, forwardLaunch.shaft) annotation(Line(points = {{-175, -110}, {-172, -110}, {-172, -149}, {-62, -149}, {-62, -110}, {-65, -110}}, color = {0, 0, 0}));
  connect(reverseStandstill.flange, reverseLaunch.shaft) annotation(Line(points = {{125, -110}, {128, -110}, {128, -149}, {238, -149}, {238, -110}, {235, -110}}, color = {0, 0, 0}));

  assert(forwardMotoringPower >= -1e-8 and reverseMotoringPower >= -1e-8,
    "Motoring must deliver mechanical power in either direction");
  assert(forwardMotoringPower <= positivePower.k + 1e-6 and reverseMotoringPower <= positivePower.k + 1e-6,
    "Motoring mechanical output must not exceed supplied electrical power");
  assert(forwardRegenPower <= 1e-8 and reverseRegenPower <= 1e-8,
    "Regeneration must absorb mechanical power in either direction");
  assert(forwardLaunchTorque >= -1e-8 and reverseLaunchTorque <= 1e-8,
    "Standstill torque must respect the configured launch direction");

  when terminal() then
    assert(forwardMotoringTorque > 0 and reverseMotoringTorque < 0 and
      forwardRegenTorque < 0 and reverseRegenTorque > 0,
      "All four motor operating quadrants must produce nonzero torque with the correct sign");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Interval = 0.001, Tolerance = 1e-8),
    Documentation(info = "<html>
<p>
Regression fixture for motor power/torque sign behavior at positive speed,
negative speed, and standstill. It covers motoring, regeneration, and the
configurable forward/reverse launch convention.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-360, -160}, {260, 240}}, preserveAspectRatio = true)));
end TestPowerLimitedMotor;
