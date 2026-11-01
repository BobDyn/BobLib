within BobLib.Records.VehicleRecord.Chassis.Suspension;

record AxleDW_BCRecord

  import Modelica.Units.SI;

  // Geometry
  parameter SI.Position bellcrankPivot[3] "Vector from origin to bellcrank pivot, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankPivotAxis[3] "Unit vector along bellcrank pivot axis, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankRodPickup[3] "Left bellcrank push/pullrod attachment coordinates from origin, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankShockPickup[3] "Left bellcrank shock attachment coordinates from origin, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter Integer rodPickup "Push/pullrod pickup ordering metadata; unused by the two-pickup bellcrank model" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter Integer shockPickup "Shock pickup ordering metadata; unused by the two-pickup bellcrank model" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter Boolean rodToLower "Whether push/pullrod mounts to lower wishbone. False if mounted to upper wishbone" annotation(choices(checkBox = true),
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter SI.Position rodMount[3] "Vector from origin to push/pullrod wishbone mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position shockMount[3] "Vector from origin to shock chassis mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter Integer springPoints(min = 2, max = size(springTable, 1)) = size(springTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Spring Params"));
  parameter Real springTable[:, 2] "Table of spring force [N] vs deflection [m], [dx1, F1; dx2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter SI.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter Integer damperPoints(min = 2, max = size(damperTable, 1)) = size(damperTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Damper Params"));
  parameter Real damperTable[:, 2] "Table of damper force [N] vs relative velocity [m/s], [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Damper Params"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_BCRecord</code> stores double-wishbone axle parameters with bellcrank-actuated spring and damper motion.
</p>
<p>
It holds the bellcrank pivot and pickups, the push/pullrod and shock chassis mounts, and the tabular spring and damper data.
</p>
<p>
Steering-rack data, wheel data, mass properties, and double-wishbone hardpoints
are supplied by the separate <code>RackAndPinionRecord</code>,
<code>PartialWheelRecord</code>, <code>AxleMassRecord</code>, and
<code>WishboneUprightLoopRecord</code> records that the vehicle definition binds
alongside this one.
</p>
<p>
Pickup coordinates specify physical attachment locations: the push/pullrod connects
to bellcrank pickupFrame1, the shock to pickupFrame2. Each connector follows its corresponding named
pickup coordinates through a fixed translation. The axle mirrors the left-side
coordinates to the right side; changing an attachment location requires changing
its named coordinates, not editing connections.
</p>
<p>
The rodPickup and shockPickup indices retain pickup ordering metadata for
compatibility. The two-pickup bellcrank model does not use them for geometry,
connections, or animation.
</p>
</html>"));
end AxleDW_BCRecord;
