within BobLibTest.Experiments.Standards.Configurations;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrank,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC;
