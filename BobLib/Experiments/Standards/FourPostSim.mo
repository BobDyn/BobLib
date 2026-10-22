within BobLib.Experiments.Standards;

model FourPostSim

  extends Templates.FourPost.FourPostSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar;
  extends BobLib.Icons.SimulationIcon;

  annotation(
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html><p>Generated standard four-post simulation entry point.</p></html>"));
end FourPostSim;
