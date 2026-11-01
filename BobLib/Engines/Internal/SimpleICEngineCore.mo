within BobLib.Engines.Internal;
block SimpleICEngineCore

  "Simple throttle-to-torque law for an internal-combustion engine sample"
  extends Modelica.Blocks.Icons.Block;

  parameter BobLib.Records.VehicleRecord.Engines.SimpleICEngineRecord pEngine
    "Engine torque, inertia and response calibration" annotation(Dialog(group = "Calibration"));

  Modelica.Blocks.Interfaces.RealInput throttle
    "Driver accelerator request, clamped internally to 0..1" annotation(
      Placement(
        transformation(origin = {0, 0}, extent = {{-120, 40}, {-80, 80}}),
        iconTransformation(origin = {-20, 0}, extent = {{-120, 40}, {-80, 80}})));

  Modelica.Blocks.Interfaces.RealInput w(unit = "rad/s")
    "Engine shaft speed" annotation(
      Placement(
        transformation(origin = {0, 0}, extent = {{-120, -80}, {-80, -40}}),
        iconTransformation(origin = {-20, 0}, extent = {{-120, -80}, {-80, -40}})));

  Modelica.Blocks.Interfaces.RealOutput tau(unit = "N.m")
    "Commanded crankshaft torque" annotation(
      Placement(
        transformation(origin = {-120, 0}, extent = {{200, -20}, {240, 20}}),
        iconTransformation(extent = {{100, -10}, {120, 10}})));

protected
  Real throttleLimited(unit = "1");
  Real redlineScale(unit = "1");

equation
  throttleLimited = noEvent(max(0, min(1, throttle)));
  redlineScale = noEvent(max(0, min(1,
    (pEngine.redlineSpeed + pEngine.redlineSoftWidth - abs(w)) / max(pEngine.redlineSoftWidth, 1e-6))));
  tau = pEngine.peakTorque*throttleLimited*redlineScale
    - pEngine.frictionTorque*Modelica.Math.tanh(w / max(pEngine.w_eps, 1e-6));

  annotation(Documentation(info = "<html>
<p>
Block <code>SimpleICEngineCore</code> is intentionally small: it clamps driver
accelerator request, rolls torque off at redline, and applies a smooth
speed-opposing friction torque. It is a sample core for architecture and
interface work, not a calibrated combustion model.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)));
end SimpleICEngineCore;