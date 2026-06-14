within BobLibVehicleInterfacesTests.Regression;

model VehicleSimAnimationOn
  extends BobLibVehicleInterfaces.Experiments.Standards.VehicleSim(headless = false);

  annotation(
    experiment(StartTime = 0, StopTime = 0.02, Tolerance = 1e-06, Interval = 0.002));
end VehicleSimAnimationOn;
