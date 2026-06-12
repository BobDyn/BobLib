within BobLib.Standards.Templates;

model VehicleSim_DWBCStabar_DWDirect
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWDirectRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBCStabar_DWDirect);
end VehicleSim_DWBCStabar_DWDirect;
