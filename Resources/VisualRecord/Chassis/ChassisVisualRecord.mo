within BobLib.Resources.VisualRecord.Chassis;

record ChassisVisualRecord
  import BobLib.Resources.VisualRecord.Chassis.Suspension.AxleDWBaseVisualRecord;

  AxleDWBaseVisualRecord frontAxle;
  AxleDWBaseVisualRecord rearAxle;
  
end ChassisVisualRecord;
