within BobLibVehicleInterfaces.Experiments.Standards;

model FourPostSim

  extends Templates.FourPost.FourPostSim_DWBCStabar_DWBCStabar;
  extends BobLibVehicleInterfaces.Icons.SimulationIcon;

  annotation(
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Model <code>FourPostSim</code> is the front-facing standard four-post simulation entry point.
</p>
<p>
It extends the selected four-post architecture template, applies the standard experiment setup, and is intended for suspension and chassis response studies.
</p>
</html>"));
end FourPostSim;
