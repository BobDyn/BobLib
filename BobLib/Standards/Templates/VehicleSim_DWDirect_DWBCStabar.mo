within BobLib.Standards.Templates;

model VehicleSim_DWDirect_DWBCStabar
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWDirect_DWBCStabarRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWDirect_DWBCStabar);
end VehicleSim_DWDirect_DWBCStabar;
