within BobLib.Experiments.Standards.Examples;
model RoadAndAtmosphereVehicle "Native VI driver with road friction and wind replacement"

  extends BobLib.Experiments.Standards.VehicleSim(
    redeclare VehicleInterfaces.Roads.FlatRoad road(mu = 0.35),
    redeclare BobLib.Atmospheres.ConstantAtmosphere atmosphere(v = {-3, 2, 0}, T = 303.15));
  annotation(experiment(StopTime = 2, Tolerance = 1e-6, Interval = 0.002),
    Documentation(info = "<html><p>A runnable native-driver vehicle with VI road and atmosphere replacements. Road friction scales the MF52 reference friction; wind is expressed in world coordinates and sampled at the aero load point. This example uses a horizontal road and the standard vehicle's initial pose. Curved roads additionally require a compatible initial pose and locally unique surface projection.</p></html>"));
end RoadAndAtmosphereVehicle;
