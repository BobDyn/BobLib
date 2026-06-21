within BobLibVehicleInterfaces.Experiments.Standards.Templates.Vehicle;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect

  extends BaseVehicleSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord,
    redeclare BobLibVehicleInterfaces.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord,
      redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar,
      redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_Direct,
      pFrStabar(
        leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
        barRate = pVehicle.pFrStabar.barRate),
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect;
