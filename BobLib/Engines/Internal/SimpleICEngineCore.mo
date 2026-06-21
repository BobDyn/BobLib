within BobLib.Engines.Internal;
block SimpleICEngineCore

  "Simple throttle-to-torque law for an internal-combustion engine sample"
  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

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

  Modelica.Blocks.Interfaces.RealInput throttle
    "Driver accelerator request, clamped internally to 0..1" annotation(
      Placement(
        transformation(origin = {-20, 0}, extent = {{-120, 40}, {-80, 80}}),
        iconTransformation(origin = {-20, 0}, extent = {{-120, 40}, {-80, 80}})));

  Modelica.Blocks.Interfaces.RealInput w(unit = "rad/s")
    "Engine shaft speed" annotation(
      Placement(
        transformation(origin = {-20, 0}, extent = {{-120, -80}, {-80, -40}}),
        iconTransformation(origin = {-20, 0}, extent = {{-120, -80}, {-80, -40}})));

  Modelica.Blocks.Interfaces.RealOutput tau(unit = "N.m")
    "Commanded crankshaft torque" annotation(
      Placement(
        transformation(origin = {-100, 0}, extent = {{200, -20}, {240, 20}}),
        iconTransformation(extent = {{100, -10}, {120, 10}})));

protected
  Real throttleLimited(unit = "1");
  Real redlineScale(unit = "1");

equation
  throttleLimited = noEvent(max(0, min(1, throttle)));
  redlineScale = noEvent(max(0, min(1,
    (redlineSpeed + redlineSoftWidth - abs(w)) / max(redlineSoftWidth, 1e-6))));
  tau = peakTorque*throttleLimited*redlineScale
    - frictionTorque*Modelica.Math.tanh(w / max(w_eps, 1e-6));

  annotation(Documentation(info = "<html>
<p>
Block <code>SimpleICEngineCore</code> is intentionally small: it clamps driver
accelerator request, rolls torque off at redline, and applies a smooth
speed-opposing friction torque. It is a sample core for architecture and
interface work, not a calibrated combustion model.
</p>
</html>"));
end SimpleICEngineCore;