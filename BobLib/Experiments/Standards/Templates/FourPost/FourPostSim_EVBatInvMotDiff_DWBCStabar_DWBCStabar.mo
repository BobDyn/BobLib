within BobLib.Experiments.Standards.Templates.FourPost;

model FourPostSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar,
    redeclare model RrAxleModel = BobLib.Chassis.Suspension.RrAxleDW_BC_Stabar,
    frAxleDW(
      pStabar = pFrStabar
    ),
    pFrStabar(
      leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
      barRate = 0
    ),
    rrAxleDW(
      pStabar = pRrStabar
    ),
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
      barRate = 0
    )
  );
  annotation(
    Documentation(info = "<html>
<p>
Generated four-post template <code>FourPostSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar</code> binds the selected vehicle record and axle architecture.
</p>
</html>"));
end FourPostSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar;
