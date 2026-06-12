within BobLib.Standards.Templates;

model FourPostSim_DWDirect_DWBC
  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWDirect_DWBCRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_Direct,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_BC);
end FourPostSim_DWDirect_DWBC;
