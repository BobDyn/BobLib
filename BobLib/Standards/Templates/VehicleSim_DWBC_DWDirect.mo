within BobLib.Standards.Templates;

model VehicleSim_DWBC_DWDirect

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWDirectRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBC_DWDirect);
end VehicleSim_DWBC_DWDirect;
