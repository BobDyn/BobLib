within BobLibTest.Experiments.Standards.Examples;
model TestSpatialBrakingVehicle

  extends BobLib.Experiments.Standards.Examples.SpatialBrakingVehicle(vehicle(headless = true));

equation
  when terminal() then
    assert(vehicle.velX<14.9 and vehicle.velX>0, "Spatial brake step must slow the vehicle without reversing");
  end when;
  annotation(experiment(StopTime = 1, Tolerance = 1e-6));
end TestSpatialBrakingVehicle;
