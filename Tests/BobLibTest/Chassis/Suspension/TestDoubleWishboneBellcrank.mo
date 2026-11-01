within BobLibTest.Chassis.Suspension;

model TestDoubleWishboneBellcrank

  extends BobLibTest.Experiments.Standards.Configurations.FourPostSim_DWBC_DWBC;
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6));
  annotation(Diagram(coordinateSystem(extent = {{-180, -160}, {180, 140}}, preserveAspectRatio = true)));
end TestDoubleWishboneBellcrank;
