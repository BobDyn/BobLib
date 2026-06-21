within BobLib.Standards.Templates;

model FourPostSim_DWBC_DWBCStabar

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWBCStabarRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_BC_Stabar,
    pRrStabar(
      leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
      barRate = 0));
end FourPostSim_DWBC_DWBCStabar;
