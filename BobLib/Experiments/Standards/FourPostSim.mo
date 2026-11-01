within BobLib.Experiments.Standards;

model FourPostSim

  extends BaseClasses.PartialDoubleWishboneFourPost(
    redeclare replaceable record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare replaceable model FrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    redeclare replaceable model RrAxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar,
    frAxleDW(pAxle(stabilizer = pFrStabar)),
    rrAxleDW(pAxle(stabilizer = pRrStabar)),
    pFrStabar(
      leftArmEnd = pVehicle.chassis.frontAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.frontAxle.stabilizer.leftBarEnd,
      barRate = 0),
    pRrStabar(
      leftArmEnd = pVehicle.chassis.rearAxle.stabilizer.leftArmEnd,
      leftBarEnd = pVehicle.chassis.rearAxle.stabilizer.leftBarEnd,
      barRate = 0));
  extends BobLib.Icons.SimulationIcon;

  annotation(
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      nls = "kinsol",
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
<h4>Numerical solver</h4><p>OMEdit selects <code>nls=kinsol</code> for the algebraic equilibrium equations. Pass <code>-nls=kinsol</code> explicitly when launching a native executable outside OMEdit. This is independent of the time integrator: the quasi-static rig has zero continuous states and OpenModelica uses Euler even if DASSL is requested. Wheel compliance uses the elastic implementation; body inertia and suspension damping are omitted by the rig.</p><p>With OpenModelica 1.27.1 on Linux ARM64, three complete compliant four-post runs gave median wall times of 6.071 s with homotopy and 1.125 s with KINSOL; the seven-angle compliant single-axle sweep gave 2.026 s with KINSOL. Full MAT output and existing tolerances were retained. Across both protocols, 36,414 sampled output/deflection comparisons agreed within 1.3e-9 maximum absolute difference. These measurements cover the tested bellcrank/stabilizer configuration, not every architecture or calibration, and are not real-time guarantees.</p></html>"));
end FourPostSim;
