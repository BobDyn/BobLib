within BobLib.Standards.Templates;

model VehicleSim_DWBC_DWBC
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWBCRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBC_DWBC);
end VehicleSim_DWBC_DWBC;
