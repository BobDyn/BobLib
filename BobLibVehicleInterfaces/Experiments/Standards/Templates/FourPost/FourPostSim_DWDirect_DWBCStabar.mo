within BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPost;

model FourPostSim_DWDirect_DWBCStabar

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWDirect_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_Direct,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar,
    rrAxleDW(pStabar = pRrStabar),
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
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
