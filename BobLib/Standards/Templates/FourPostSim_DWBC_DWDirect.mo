within BobLib.Standards.Templates;

model FourPostSim_DWBC_DWDirect

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_Direct);
end FourPostSim_DWBC_DWDirect;
