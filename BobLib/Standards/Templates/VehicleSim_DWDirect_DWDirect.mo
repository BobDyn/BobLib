within BobLib.Standards.Templates;

model VehicleSim_DWDirect_DWDirect
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWDirect_DWDirectRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWDirect_DWDirect);
end VehicleSim_DWDirect_DWDirect;
