within BobLib.Standards.Templates;

model VehicleSim_DWBC_DWBCStabar

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWBCStabarRecord,
    redeclare model VehicleModel = BobLib.Vehicle.Vehicle_DWBC_DWBCStabar);
end VehicleSim_DWBC_DWBCStabar;
