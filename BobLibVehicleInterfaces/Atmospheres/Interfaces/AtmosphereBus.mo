within BobLibVehicleInterfaces.Atmospheres.Interfaces;
expandable connector AtmosphereBus

  "BobLib atmosphere signal bus"
  extends Modelica.Icons.SignalSubBus;

  import SI = Modelica.Units.SI;

  SI.Velocity windVelocityWorld[3]
    "Wind velocity resolved in world frame";
  SI.Density airDensity
    "Local air density";
  SI.Temperature airTemperature
    "Local air temperature";
  Real relativeHumidity(unit = "1")
    "Local relative humidity";
  SI.AbsolutePressure pressure
    "Local ambient pressure";

  annotation(
    defaultComponentPrefixes = "protected",
    Documentation(info = "<html>
<p>
Expandable sub-bus for atmosphere-owned measurements published through the
shared BobLib atmosphere signal namespace. The BobLib convention is for a
vehicle assembly to own one <code>AtmosphereBus</code>, connect the atmosphere
publisher to it, and connect aero or telemetry subscribers to the same bus.
</p>
</html>"));
end AtmosphereBus;
