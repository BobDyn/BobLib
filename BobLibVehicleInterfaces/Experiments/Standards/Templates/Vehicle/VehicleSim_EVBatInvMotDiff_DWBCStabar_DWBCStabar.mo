within BobLibVehicleInterfaces.Experiments.Standards.Templates.Vehicle;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare BobLibVehicleInterfaces.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
      redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar,
      redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar,
      pFrStabar(
        leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
        barRate = pVehicle.pFrStabar.barRate),
      pRrStabar(
        leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
        barRate = pVehicle.pRrStabar.barRate),
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar;
