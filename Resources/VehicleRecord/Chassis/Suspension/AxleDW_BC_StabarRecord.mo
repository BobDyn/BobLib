within BobLib.Resources.VehicleRecord.Chassis.Suspension;

record AxleDW_BC_StabarRecord
  import Modelica.SIunits;
  
  // Geometry
  parameter SIunits.Position bellcrankPivot[3] "Vector from origin to bellcrank pivot, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position bellcrankPivotAxis[3] "Unit vector along bellcrank pivot axis, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position bellcrankRodPickup[3] "Vector from origin to push/pullrod pickup, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position bellcrankShockPickup[3] "Vector from origin to shock pickup, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position bellcrankStabarPickup[3] "Vector from origin to bellcrank pickup, resolved in world frame" annotation(
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
  parameter SIunits.Position shockMount[3] "Vector from origin to shock chassis mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position springTable[:,2] "Table of spring force vs deflection, [dx1, F1; dx1, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SIunits.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SIunits.Position damperTable[:,2] "Table of damper force vs velocity, [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  
end AxleDW_BC_StabarRecord;
