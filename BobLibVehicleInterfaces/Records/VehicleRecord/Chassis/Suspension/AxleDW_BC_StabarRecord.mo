within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension;

record AxleDW_BC_StabarRecord
  import SI = Modelica.Units.SI;

  // Geometry
  parameter SI.Position bellcrankPivot[3] "Vector from origin to bellcrank pivot, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankPivotAxis[3] "Unit vector along bellcrank pivot axis, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankRodPickup[3] "Vector from origin to push/pullrod pickup, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankShockPickup[3] "Vector from origin to shock pickup, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position bellcrankStabarPickup[3] "Vector from origin to bellcrank pickup, resolved in world frame" annotation(
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
  parameter SI.Position rodMount[3] "Vector from origin to push/pullrod wishbone mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position shockMount[3] "Vector from origin to shock chassis mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position springTable[:,2] "Table of spring force vs deflection, [dx1, F1; dx1, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SI.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SI.Position damperTable[:,2] "Table of damper force vs velocity, [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_BC_StabarRecord</code> stores double-wishbone axle parameters with bellcrank actuation and a stabilizer bar.
</p>
<p>
It combines bellcrank pickups, spring and damper data, stabilizer-bar geometry, steering rack data, and double-wishbone hardpoints.
</p>
</html>"));
end AxleDW_BC_StabarRecord;
