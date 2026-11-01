within BobLib.Experiments.Standards.Examples;

model MixedSuspensionVehicle

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBC_DWDirectRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBC_DWDirectRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrank,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end MixedSuspensionVehicle;
