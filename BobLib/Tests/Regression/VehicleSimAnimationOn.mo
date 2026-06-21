within BobLib.Tests.Regression;

model VehicleSimAnimationOn

  extends BobLib.Standards.VehicleSim(enableAnimation = true);

  annotation(
    experiment(StartTime = 0, StopTime = 0.02, Tolerance = 1e-06, Interval = 0.002));
end VehicleSimAnimationOn;
