within BobLib.Records.VisualRecord.Chassis;

record ChassisVisualRecord

  import BobLib.Records.VisualRecord.Chassis.Suspension.AxleDW_BC_ARB_VisualRecord;

  AxleDW_BC_ARB_VisualRecord frontAxle;
  AxleDW_BC_ARB_VisualRecord rearAxle;

  annotation(
    Documentation(info = "<html>
<p>
Record <code>ChassisVisualRecord</code> stores visualization and diagram geometry data.
</p>
<p>
It supports clean icon, diagram, and animation presentation without coupling drawing choices to the physical equations.
</p>
</html>"));
end ChassisVisualRecord;
