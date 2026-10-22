within BobLibTest.Experiments.Standards.Configurations;

model FourPostSim_DWDirect_DWDirect

  extends BobLib.Experiments.Standards.BaseClasses.PartialDoubleWishboneFourPost(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneDirect);
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWDirect_DWDirect</code> specializes the four-post template with front direct-acting double wishbone suspension and rear direct-acting double wishbone suspension.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWDirect_DWDirect;
