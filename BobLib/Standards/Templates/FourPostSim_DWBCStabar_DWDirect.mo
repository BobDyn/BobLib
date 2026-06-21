within BobLib.Standards.Templates;

model FourPostSim_DWBCStabar_DWDirect

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_BC_Stabar,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_Direct,
    pFrStabar(
      leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
      leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
      barRate = 0));
end FourPostSim_DWBCStabar_DWDirect;
