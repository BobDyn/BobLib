within BobLibTest.Experiments.Standards;

model TestVehicleSIL "Car-only assembly coverage; prescribed-command acceptance runs through FMI"

  extends BobLib.Experiments.Standards.VehicleSIL;
  annotation(Documentation(info = "<html><p>Integration coverage for the SIL plant. The exported Co-Simulation artifact is exercised by Tools/verify_sil.py.</p></html>"));
end TestVehicleSIL;
