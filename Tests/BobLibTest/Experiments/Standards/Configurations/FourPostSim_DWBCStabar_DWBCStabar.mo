within BobLibTest.Experiments.Standards.Configurations;

model FourPostSim_DWBCStabar_DWBCStabar

  extends BobLib.Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    frAxleDW(pAxle(stabilizer = pFrStabar)),
    rrAxleDW(pAxle(stabilizer = pRrStabar)),
    pFrStabar(
      leftArmEnd = pVehicle.chassis.frontAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.frontAxle.stabilizer.leftBarEnd,
      barRate = 0),
    pRrStabar(
      leftArmEnd = pVehicle.chassis.rearAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.rearAxle.stabilizer.leftBarEnd,
      barRate = 0));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWBCStabar_DWBCStabar</code> specializes the four-post template with front bellcrank-actuated double wishbone suspension with a stabilizer bar and rear bellcrank-actuated double wishbone suspension with a stabilizer bar.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWBCStabar_DWBCStabar;
