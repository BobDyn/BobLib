within BobLib.Experiments.Standards.Templates.Vehicle;

model VehicleSim_EVBatInvMotDiff_DWDirect_DWBC

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCRecord,
    redeclare BobLib.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.FrAxleDW_Direct,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.RrAxleDW_BC,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
end VehicleSim_EVBatInvMotDiff_DWDirect_DWBC;
