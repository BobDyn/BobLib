within BobLib.Standards;

model FourPostSim

  extends Templates.FourPostSim_DWBCStabar_DWBCStabar;
  annotation(
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"));
end FourPostSim;
