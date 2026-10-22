within BobLibTest.Experiments.Standards.Configurations;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect;
