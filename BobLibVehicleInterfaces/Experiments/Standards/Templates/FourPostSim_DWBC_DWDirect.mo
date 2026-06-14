within BobLibVehicleInterfaces.Experiments.Standards.Templates;

model FourPostSim_DWBC_DWDirect
  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.DWBC_DWDirectRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostRrAxleDW_Direct);
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWBC_DWDirect</code> specializes the four-post template with front bellcrank-actuated double wishbone suspension and rear direct-acting double wishbone suspension.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWBC_DWDirect;
