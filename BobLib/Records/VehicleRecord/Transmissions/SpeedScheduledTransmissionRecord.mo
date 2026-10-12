within BobLib.Records.VehicleRecord.Transmissions;

record SpeedScheduledTransmissionRecord

  import SI = Modelica.Units.SI;

  parameter Integer nGears(min = 1) = 1
    "Number of forward gears";
  parameter Real gearRatios[nGears] = {3.31}
    "Gear ratios ordered from lowest gear (highest ratio) to highest gear";
  parameter SI.Velocity upshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear upshifts to the next";
  parameter SI.Velocity downshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear downshifts to the previous";
  parameter Integer initialGear(min = 1, max = nGears) = 1
    "Gear engaged at initialization";
  parameter SI.Radius wheelRadius(min = 1e-6) = 0.2045
    "Loaded wheel radius used to estimate vehicle speed from output speed";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>SpeedScheduledTransmissionRecord</code> contains the
vehicle-level parameters passed to
<code>Transmissions.SpeedScheduledTransmission</code>.
</p>
<p>
Option A gearing: <code>gearRatios</code> holds the complete motor-to-wheel
reduction per gear (primary x gear x sprocket), ordered from lowest gear
(highest numerical ratio) to highest gear. The <code>upshiftSpeeds</code> and
<code>downshiftSpeeds</code> vectors carry one entry per gear boundary
(<code>nGears - 1</code> entries); a downshift speed below the matching upshift
speed provides shift hysteresis.
</p>
</html>"));
end SpeedScheduledTransmissionRecord;
