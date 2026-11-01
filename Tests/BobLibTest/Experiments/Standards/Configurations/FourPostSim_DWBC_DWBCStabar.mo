within BobLibTest.Experiments.Standards.Configurations;

model FourPostSim_DWBC_DWBCStabar

  extends BobLib.Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBC_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrank,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    rrAxleDW(pAxle(stabilizer = pRrStabar)),
    pRrStabar(
      leftArmEnd = pVehicle.chassis.rearAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.rearAxle.stabilizer.leftBarEnd,
      barRate = 0));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWBC_DWBCStabar</code> specializes the four-post template with front bellcrank-actuated double wishbone suspension and rear bellcrank-actuated double wishbone suspension with a stabilizer bar.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWBC_DWBCStabar;
