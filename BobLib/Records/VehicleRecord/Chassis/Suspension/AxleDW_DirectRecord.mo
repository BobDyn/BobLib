within BobLib.Records.VehicleRecord.Chassis.Suspension;

record AxleDW_DirectRecord

  import SI = Modelica.Units.SI;

  // Geometry
  parameter Boolean rodToLower "Whether push/pullrod mounts to lower wishbone. False if mounted to upper wishbone" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter SI.Position rodMount[3] "Vector from origin to push/pullrod wishbone mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position shockMount[3] "Vector from origin to shock chassis mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position springTable[:, 2] "Table of spring force [N] vs deflection [m], [dx1, F1; dx2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter SI.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter SI.Position damperTable[:, 2] "Table of damper force [N] vs relative velocity [m/s], [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Damper Params"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_DirectRecord</code> stores double-wishbone axle parameters for a direct spring and damper layout.
</p>
<p>
It holds the push/pullrod and shock chassis mounts and the tabular spring and damper data.
</p>
<p>
Steering-rack data, wheel data, mass properties, and double-wishbone hardpoints
are supplied by the separate <code>RackAndPinionRecord</code>,
<code>PartialWheelRecord</code>, <code>AxleMassRecord</code>, and
<code>WishboneUprightLoopRecord</code> records that the vehicle definition binds
alongside this one.
</p>
</html>"));
end AxleDW_DirectRecord;
