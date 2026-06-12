within BobLib.Standards.Templates;

model FourPostSim_DWBCStabar_DWBCStabar
  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_BC_Stabar,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_BC_Stabar,
    pFrStabar(
      leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
      barRate = 0),
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
      barRate = 0));
end FourPostSim_DWBCStabar_DWBCStabar;
