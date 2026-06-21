within BobLib.Standards.Templates;

model FourPostSim_DWBC_DWBC

  extends BaseFourPostSim(
    redeclare record VehicleRecord = BobLib.Resources.VehicleDefn.DWBC_DWBCRecord,
    redeclare model FrAxleModel = BobLib.Standards.Templates.FourPostFrAxleDW_BC,
    redeclare model RrAxleModel = BobLib.Standards.Templates.FourPostRrAxleDW_BC);
end FourPostSim_DWBC_DWBC;
