within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates;

record PartialWheelRecord

  import SI = Modelica.Units.SI;

  // Dimensions
  parameter SI.Length R0 "Tire unloaded static radius" annotation(
    Dialog(group = "Dimensions"));
  parameter SI.Length rimR0 = R0*0.625 "Rim unloaded static radius" annotation(
    Dialog(group = "Dimensions"));
  parameter SI.Length rimWidth = rimR0*1.4 "Rim unloaded width" annotation(
    Dialog(group = "Dimensions"));
  parameter SI.Angle staticAlpha "Static toe angle in DEGREES, following Z-up convention" annotation(
    Dialog(group = "Attitude"));
  parameter SI.Angle staticGamma "Static inclination angle in DEGREES, following Z-up convention" annotation(
    Dialog(group = "Attitude"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>PartialWheelRecord</code> stores wheel geometry and static alignment data.
</p>
<p>
Wheel radius, rim dimensions, static slip angle, and camber are shared by wheel
physics, tire models, and chassis-level contact-patch wiring.
</p>
</html>"));
end PartialWheelRecord;
