within BobLib.Standards.Templates;

model VehicleSim_DWBCStabar_DWBCStabar
  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBCStabar_DWBCStabar);
end VehicleSim_DWBCStabar_DWBCStabar;
