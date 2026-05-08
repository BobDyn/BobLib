within BobLib.Resources.VehicleRecord.Chassis.Suspension;

record AxleDW_DirectRecord
  import Modelica.SIunits;
  
  // Geometry
  parameter SIunits.Position shockMount[3] "Vector from origin to shock chassis mount, resolved in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position springTable[:,2] "Table of spring force vs deflection, [dx1, F1; dx1, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SIunits.Length springFreeLength "Spring free length (zero-force length) = installed length + static compression" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SIunits.Position damperTable[:,2] "Table of damper force vs velocity, [v1, F1; v2, F2; ...]" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  
end AxleDW_DirectRecord;
