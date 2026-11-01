within BobLibTest.Experiments.Standards.Configurations;

model VehicleSim_EVBatInvMotDiff_DWDirect_DWBC

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrank,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end VehicleSim_EVBatInvMotDiff_DWDirect_DWBC;
