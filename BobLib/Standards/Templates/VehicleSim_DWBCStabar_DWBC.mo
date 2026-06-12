within BobLib.Standards.Templates;

model VehicleSim_DWBCStabar_DWBC
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWBCRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBCStabar_DWBC);
end VehicleSim_DWBCStabar_DWBC;
