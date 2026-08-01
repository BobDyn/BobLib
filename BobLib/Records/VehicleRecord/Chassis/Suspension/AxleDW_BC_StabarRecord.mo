within BobLib.Records.VehicleRecord.Chassis.Suspension;

record AxleDW_BC_StabarRecord

  import SI = Modelica.Units.SI;

  // Geometry
  parameter SI.Position bellcrankPivot[3] "Vector from origin to bellcrank pivot, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankPivotAxis[3] "Unit vector along bellcrank pivot axis, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankRodPickup[3] "Vector from origin to push/pullrod pickup, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankShockPickup[3] "Vector from origin to shock pickup, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankStabarPickup[3] "Vector from origin to stabilizer-bar droplink pickup, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter Integer rodPickup "Push/pullrod pickup mapping, where 1 is the most counter-clockwise pickup about the left bellcrank (generally with the lowest Z coordinate)" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter Integer shockPickup "Shock pickup mapping, where 1 is the most counter-clockwise pickup about the left bellcrank (generally with the lowest Z coordinate)" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter Integer stabarPickup "Stabar pickup mapping, where 1 is the most counter-clockwise pickup about the left bellcrank (generally with the lowest Z coordinate)" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter Boolean rodToLower "Whether push/pullrod mounts to lower wishbone. False if mounted to upper wishbone" annotation(
    Evaluate = true,
    Dialog(group = "Geometry"));
  parameter SI.Position rodMount[3] "Vector from origin to push/pullrod wishbone mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position shockMount[3] "Vector from origin to shock chassis mount, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter Real springTable[:, 2] "Table of spring force [N] vs deflection [m], [dx1, F1; dx2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter SI.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Spring Params"));
  parameter SI.Position damperTable[:, 2] "Table of damper force [N] vs relative velocity [m/s], [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Damper Params"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_BC_StabarRecord</code> stores double-wishbone axle parameters with bellcrank actuation and a stabilizer bar.
</p>
<p>
It holds the bellcrank pivot and pickups, the push/pullrod and shock chassis mounts, and the tabular spring and damper data.
</p>
<p>
Stabilizer-bar geometry, steering-rack data, wheel data, mass properties, and
double-wishbone hardpoints are supplied by the separate
<code>StabarRecord</code>, <code>RackAndPinionRecord</code>,
<code>PartialWheelRecord</code>, <code>AxleMassRecord</code>, and
<code>WishboneUprightLoopRecord</code> records that the vehicle definition binds
alongside this one.
</p>
</html>"));
end AxleDW_BC_StabarRecord;
