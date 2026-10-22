within BobLibTest.Experiments.Standards.Configurations;

model FourPostSim_DWDirect_DWBCStabar

  extends BobLib.Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    rrAxleDW(pAxle(stabilizer = pRrStabar)),
    pRrStabar(
      leftArmEnd = pVehicle.chassis.rearAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.rearAxle.stabilizer.leftBarEnd,
      barRate = 0));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWDirect_DWBCStabar</code> specializes the four-post template with front direct-acting double wishbone suspension and rear bellcrank-actuated double wishbone suspension with a stabilizer bar.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWDirect_DWBCStabar;
