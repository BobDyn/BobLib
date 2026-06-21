within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52;

record SetupRecord

  import SI = Modelica.Units.SI;

  parameter SI.Force FNOMIN
    "Nominal vertical load used for normalization (Fz0 in MF formulations)";

  parameter SI.Force FZMIN = 1e-3
    "Minimum vertical load used for Magic Formula evaluation";

  parameter SI.Force FZMAX = 1e60
    "Maximum vertical load from the tire property fit";

  parameter SI.Length UNLOADED_RADIUS
    "Unloaded tire radius (used for kinematics and moment calculations)";

end SetupRecord;
