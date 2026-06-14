within BobLibVehicleInterfaces.Experiments.Standards.Templates;

model FourPostSim_DWBC_DWBCStabar
  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.DWBC_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPostRrAxleDW_BC_Stabar,
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
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
