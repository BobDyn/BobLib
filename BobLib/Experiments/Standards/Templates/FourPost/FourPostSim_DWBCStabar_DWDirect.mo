within BobLib.Experiments.Standards.Templates.FourPost;

model FourPostSim_DWBCStabar_DWDirect

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.RrAxleDW_Direct,
    frAxleDW(pStabar = pFrStabar),
    pFrStabar(
      leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
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
