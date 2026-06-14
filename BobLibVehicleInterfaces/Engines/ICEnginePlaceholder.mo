within BobLibVehicleInterfaces.Engines;
model ICEnginePlaceholder
  "Internal-combustion engine placeholder using the VehicleInterfaces engine contract"
  extends VehicleInterfaces.Engines.NullEngine;

  annotation(
    Icon(graphics = {
      Text(extent = {{-74, 90}, {74, 60}}, textString = "IC", textColor = {78, 161, 255}, fontName = "DejaVu Sans")}),
    Documentation(info = "<html>
<p>
Model <code>ICEnginePlaceholder</code> is a compile-ready reference point for a
future internal-combustion engine subsystem.
</p>
<p>
It intentionally reuses <code>VehicleInterfaces.Engines.NullEngine</code> so the
standard accessory flange, transmission flange, optional mount, optional pedal,
and control-bus interface are visible without adding placeholder combustion
physics. The current behavior is a rigid pass-through between the accessory and
transmission flanges.
</p>
<p>
When BobLib gains a real IC engine model, keep this class as the first-level
VehicleInterfaces-facing adapter and move throttle maps, flywheel dynamics,
fuel-rate calculations, torque limits, and engine-block reactions into nested
implementation packages below <code>Engines</code>.
</p>
</html>"));
end ICEnginePlaceholder;
