within BobLibTest.Experiments.Standards.Configurations;

model VehicleSim_EVBatInvMotDiff_DWDirect_DWBCStabar

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCStabarRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCStabarRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end VehicleSim_EVBatInvMotDiff_DWDirect_DWBCStabar;
