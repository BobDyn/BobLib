within BobLib.Standards.Templates;

model VehicleSim_DWDirect_DWBC
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWDirect_DWBCRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWDirect_DWBC);
end VehicleSim_DWDirect_DWBC;
