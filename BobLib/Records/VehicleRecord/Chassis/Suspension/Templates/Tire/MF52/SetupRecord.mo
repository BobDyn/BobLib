within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52;

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

  annotation(
    Documentation(info = "<html>
<p>
Record <code>SetupRecord</code> stores MF5.2 tire setup limits and nominal values.
</p>
<p>
The tire slip and force models use this data to clamp load ranges and configure transient slip behavior.
</p>
</html>"));
end SetupRecord;
