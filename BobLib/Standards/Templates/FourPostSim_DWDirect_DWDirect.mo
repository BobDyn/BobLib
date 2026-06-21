within BobLib.Standards.Templates;

model FourPostSim_DWDirect_DWDirect

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWDirect_DWDirectRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_Direct,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_Direct);
end FourPostSim_DWDirect_DWDirect;
