within BobLibVehicleInterfaces.Experiments.Standards.Templates;

model FourPostSim_DWBC_DWBC
  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.DWBC_DWBCRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostRrAxleDW_BC);
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostSim_DWBC_DWBC</code> specializes the four-post template with front bellcrank-actuated double wishbone suspension and rear bellcrank-actuated double wishbone suspension.
</p>
<p>
It keeps all architecture choices visible at the template level so users can either follow the redeclare pattern or hard-code a year-specific vehicle.
</p>
</html>"));
end FourPostSim_DWBC_DWBC;
