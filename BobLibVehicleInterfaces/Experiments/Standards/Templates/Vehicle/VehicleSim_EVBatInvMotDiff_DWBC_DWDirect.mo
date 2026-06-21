within BobLibVehicleInterfaces.Experiments.Standards.Templates.Vehicle;

model VehicleSim_EVBatInvMotDiff_DWBC_DWDirect

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBC_DWDirectRecord,
    redeclare BobLibVehicleInterfaces.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBC_DWDirectRecord,
      redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC,
      redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_Direct,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
end VehicleSim_EVBatInvMotDiff_DWBC_DWDirect;
