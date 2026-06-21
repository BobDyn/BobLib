within BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPost;

model FourPostSim_DWBCStabar_DWBCStabar

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar,
    redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar,
    frAxleDW(pStabar = pFrStabar),
    rrAxleDW(pStabar = pRrStabar),
    pFrStabar(
      leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
      barRate = 0),
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
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
