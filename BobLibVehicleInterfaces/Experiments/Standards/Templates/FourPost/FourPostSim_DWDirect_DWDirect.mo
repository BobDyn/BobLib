within BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPost;

model FourPostSim_DWDirect_DWDirect

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWDirectRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_Direct,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_Direct);
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
