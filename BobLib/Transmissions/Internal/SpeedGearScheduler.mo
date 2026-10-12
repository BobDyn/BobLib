within BobLib.Transmissions.Internal;
model SpeedGearScheduler

  "Selects a discrete gear ratio from vehicle speed with shift hysteresis"
  import SI = Modelica.Units.SI;

  parameter Integer nGears(min = 1) = 1
    "Number of forward gears";
  parameter Real gearRatios[nGears] = {1.0}
    "Gear ratios ordered from lowest gear (highest ratio) to highest gear";
  parameter SI.Velocity upshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear upshifts to the next";
  parameter SI.Velocity downshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear downshifts to the previous";
  parameter Integer initialGear(min = 1, max = nGears) = 1
    "Gear engaged at initialization";

  Modelica.Blocks.Interfaces.RealInput vehicleSpeed(unit = "m/s")
    "Vehicle longitudinal speed used to schedule gears" annotation(
      Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  Modelica.Blocks.Interfaces.RealOutput ratio
    "Selected gear ratio" annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Interfaces.IntegerOutput gear
    "Selected gear index" annotation(
      Placement(transformation(extent = {{100, -70}, {120, -50}})));

  discrete Integer currentGear(start = initialGear, fixed = true)
    "Latched engaged gear index";

algorithm
  when nGears > 1 and pre(currentGear) < nGears and
      vehicleSpeed > upshiftSpeeds[pre(currentGear)] then
    currentGear := pre(currentGear) + 1;
  elsewhen nGears > 1 and pre(currentGear) > 1 and
      vehicleSpeed < downshiftSpeeds[pre(currentGear) - 1] then
    currentGear := pre(currentGear) - 1;
  end when;

equation
  gear = currentGear;
  ratio = gearRatios[currentGear];

  annotation(Documentation(info = "<html>
<p>
Model <code>SpeedGearScheduler</code> maps vehicle speed to a discrete gear
ratio. Gears are ordered from lowest gear (highest numerical ratio) to highest
gear. The <code>upshiftSpeeds</code>/<code>downshiftSpeeds</code> vectors carry
one entry per gear boundary; keeping the downshift speed below the upshift
speed gives shift hysteresis and prevents chatter around a threshold.
</p>
<p>
The gear index is latched in a <code>when</code> clause so a ratio change is a
discrete event rather than a continuous function of speed.
</p>
</html>"));
end SpeedGearScheduler;
