within BobLibTest.Experiments.Standards.Examples;
model TestRoadAndAtmosphereVehicle

  extends BobLib.Experiments.Standards.Examples.RoadAndAtmosphereVehicle(headless = true);
  Modelica.Units.SI.Position aeroPoint[3] = chassis.chassisFrame.r_0 +
    Modelica.Mechanics.MultiBody.Frames.resolve1(chassis.chassisFrame.R, aeroModel.mountOffset);
  Modelica.Units.SI.Velocity relativeAirWorld[3] = der(aeroPoint) - {-3, 2, 0};
  Modelica.Units.SI.Force dragWorld[3] = Modelica.Mechanics.MultiBody.Frames.resolve1(
    chassis.chassisFrame.R, aeroModel.force + {0, 0, aeroModel.downforce});

equation
  assert(aeroModel.drag > 0, "The moving vehicle must experience nonzero aerodynamic drag");
  assert(dragWorld*relativeAirWorld <= 1e-6,
    "Drag must oppose the example's imposed wind-relative motion");
  assert(Modelica.Math.Vectors.length(cross(dragWorld, relativeAirWorld)) <
    1e-6*max(1, Modelica.Math.Vectors.length(dragWorld)*Modelica.Math.Vectors.length(relativeAirWorld)),
    "The crosswind must rotate the physical drag force away from the vehicle's travel direction");
  annotation(experiment(StopTime = 2, Tolerance = 1e-6));
end TestRoadAndAtmosphereVehicle;
