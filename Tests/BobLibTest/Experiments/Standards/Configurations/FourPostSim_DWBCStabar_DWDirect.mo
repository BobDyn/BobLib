within BobLibTest.Experiments.Standards.Configurations;

model FourPostSim_DWBCStabar_DWDirect

  extends BobLib.Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
    frAxleDW(pAxle(stabilizer = pFrStabar)),
    pFrStabar(
      leftArmEnd = pVehicle.chassis.frontAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.frontAxle.stabilizer.leftBarEnd,
      barRate = 0));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWBCStabar_DWDirect</code> specializes the four-post template with front bellcrank-actuated double wishbone suspension with a stabilizer bar and rear direct-acting double wishbone suspension.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWBCStabar_DWDirect;
