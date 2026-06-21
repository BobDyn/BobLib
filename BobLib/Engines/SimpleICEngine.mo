within BobLib.Engines;
model SimpleICEngine

  "Simple internal-combustion engine sample exposed through VehicleInterfaces"
  extends VehicleInterfaces.Icons.Engine;
  extends VehicleInterfaces.Engines.Interfaces.Base;

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  parameter SI.AngularVelocity engineSpeed_start = 1200*2*pi/60
    "Initial engine speed";
  parameter SI.Inertia flywheelInertia = 0.08
    "Sample crank/flywheel inertia";
  parameter SI.Torque peakTorque = 120
    "Peak positive engine torque at full accelerator request";
  parameter SI.Torque frictionTorque = 8
    "Simple speed-opposing friction torque magnitude";
  parameter SI.AngularVelocity redlineSpeed = 6500*2*pi/60
    "Speed where torque begins to roll off";
  parameter SI.AngularVelocity redlineSoftWidth = 300*2*pi/60
    "Speed range used for smooth redline torque cutoff";
  parameter SI.AngularVelocity w_eps = 1
    "Speed regularization for friction sign";
  parameter SI.Time torqueTimeConstant = 0.03
    "First-order torque response time";

  Modelica.Mechanics.Rotational.Components.Inertia flywheel(
    J = flywheelInertia,
    w(start = engineSpeed_start, fixed = true)) annotation(
      Placement(transformation(extent = {{-32, -10}, {-12, 10}})));

  Modelica.Mechanics.Rotational.Sources.Torque combustionTorque(
    useSupport = false) annotation(
      Placement(transformation(extent = {{-70, -30}, {-50, -10}})));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor engineSpeed annotation(
      Placement(transformation(origin = {0, 42}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sensors.TorqueSensor outputTorqueSensor annotation(
      Placement(transformation(extent = {{18, -10}, {38, 10}})));

  Modelica.Mechanics.Rotational.Sensors.PowerSensor outputPowerSensor annotation(
      Placement(transformation(extent = {{52, -10}, {72, 10}})));

  BobLib.Engines.Internal.SimpleICEngineCore core(
    peakTorque = peakTorque,
    frictionTorque = frictionTorque,
    redlineSpeed = redlineSpeed,
    redlineSoftWidth = redlineSoftWidth,
    w_eps = w_eps) annotation(
      Placement(transformation(extent = {{-64, 36}, {-36, 64}})));

  Modelica.Blocks.Continuous.FirstOrder torqueLag(
    T = torqueTimeConstant,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = 0) annotation(
      Placement(transformation(extent = {{-28, 40}, {-8, 60}})));

  output SI.AngularVelocity w "Engine shaft speed";
  output SI.Torque tau "Torque delivered through the transmission flange";
  output SI.Power P "Mechanical power delivered through the transmission flange";

protected
  VehicleInterfaces.Interfaces.EngineBus engineBus annotation(
    Placement(transformation(extent = {{-72, 70}, {-52, 90}})));
  VehicleInterfaces.Interfaces.DriverBus driverBus annotation(
    Placement(transformation(extent = {{-94, 44}, {-74, 64}})));

equation
  w = engineSpeed.w;
  tau = outputTorqueSensor.tau;
  P = outputPowerSensor.power;

  connect(controlBus.engineBus, engineBus) annotation(
    Line(points = {{-100, 60}, {-62, 60}, {-62, 80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.driverBus, driverBus) annotation(
    Line(points = {{-100, 60}, {-84, 60}, {-84, 54}}, color = {255, 204, 51}, thickness = 0.5));
  connect(driverBus.acceleratorPedalPosition, core.throttle) annotation(
    Line(points = {{-84, 54}, {-74, 54}, {-74, 58}, {-65.4, 58}}, color = {0, 0, 127}));
  connect(engineSpeed.w, core.w) annotation(
    Line(points = {{-11, 42}, {-20, 42}, {-20, 32}, {-72, 32}, {-72, 42}, {-65.4, 42}}, color = {0, 0, 127}));
  connect(engineSpeed.w, engineBus.speed) annotation(
    Line(points = {{-11, 42}, {-62, 42}, {-62, 80}}, color = {0, 0, 127}));
  connect(core.tau, torqueLag.u) annotation(
    Line(points = {{-34.6, 50}, {-30, 50}}, color = {0, 0, 127}));
  connect(torqueLag.y, combustionTorque.tau) annotation(
    Line(points = {{-7, 50}, {0, 50}, {0, 22}, {-76, 22}, {-76, -20}, {-72, -20}}, color = {0, 0, 127}));
  connect(combustionTorque.flange, flywheel.flange_a) annotation(
    Line(points = {{-50, -20}, {-42, -20}, {-42, 0}, {-32, 0}}));
  connect(accessoryFlange.flange, flywheel.flange_a) annotation(
    Line(points = {{-100, 0}, {-32, 0}}));
  connect(flywheel.flange_b, engineSpeed.flange) annotation(
    Line(points = {{-12, 0}, {8, 0}, {8, 42}}));
  connect(flywheel.flange_b, outputTorqueSensor.flange_a) annotation(
    Line(points = {{-12, 0}, {18, 0}}));
  connect(outputTorqueSensor.flange_b, outputPowerSensor.flange_a) annotation(
    Line(points = {{38, 0}, {52, 0}}));
  connect(outputPowerSensor.flange_b, transmissionFlange.flange) annotation(
    Line(points = {{72, 0}, {100, 0}}));

  annotation(Documentation(info = "<html>
<p>
Model <code>SimpleICEngine</code> is a small internal-combustion engine sample
implemented inside the VehicleInterfaces engine contract.
</p>
<p>
It reads accelerator demand from <code>controlBus.driverBus</code>, publishes
engine speed to <code>controlBus.engineBus</code>, applies a simple
throttle-to-torque law with redline roll-off and friction in
<code>Engines.Internal.SimpleICEngineCore</code>, and drives the standard
<code>transmissionFlange</code> through a flywheel inertia.
</p>
<p>
The model is intended as a readable base implementation for conventional or
hybrid architecture work. It is not a calibrated combustion model and should be
replaced by map-based torque, fuel-rate, thermal, and mounting physics when
those effects matter.
</p>
</html>"));
end SimpleICEngine;
