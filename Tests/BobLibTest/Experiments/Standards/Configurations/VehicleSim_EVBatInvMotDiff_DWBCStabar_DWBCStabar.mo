within BobLibTest.Experiments.Standards.Configurations;

model VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare BobLib.Chassis.DoubleWishbone chassis(
      redeclare record ChassisRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord.ChassisRecord,
      redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
      redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pChassis = pVehicle.chassis));
end VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar;
