within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension;

record AxleDW_DirectRecord

  import SI = Modelica.Units.SI;

  // Geometry
  parameter Boolean rodToLower "Whether push/pullrod mounts to lower wishbone. False if mounted to upper wishbone" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter SI.Position rodMount[3] "Vector from origin to push/pullrod wishbone mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position shockMount[3] "Vector from origin to shock chassis mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position springTable[:, 2] "Table of spring force vs deflection, [dx1, F1; dx1, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SI.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SI.Position damperTable[:, 2] "Table of damper force vs velocity, [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_DirectRecord</code> stores double-wishbone axle parameters for a direct spring and damper layout.
</p>
<p>
It contains shock, rack, wheel, mass, and hardpoint data consumed by the direct-acting front and rear axle models.
</p>
</html>"));
end AxleDW_DirectRecord;
