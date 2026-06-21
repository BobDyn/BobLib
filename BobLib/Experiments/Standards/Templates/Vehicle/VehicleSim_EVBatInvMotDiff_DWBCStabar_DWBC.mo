within BobLib.Experiments.Standards.Templates.Vehicle;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCRecord,
    redeclare BobLib.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.RrAxleDW_BC,
      pFrStabar(
        leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
        barRate = pVehicle.pFrStabar.barRate),
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC;
