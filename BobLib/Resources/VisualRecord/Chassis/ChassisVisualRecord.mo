within BobLib.Resources.VisualRecord.Chassis;

record ChassisVisualRecord
  import BobLib.Resources.VisualRecord.Chassis.Suspension.AxleDW_BC_ARB_VisualRecord;

  AxleDW_BC_ARB_VisualRecord frontAxle;
  AxleDW_BC_ARB_VisualRecord rearAxle;
  
end ChassisVisualRecord;
