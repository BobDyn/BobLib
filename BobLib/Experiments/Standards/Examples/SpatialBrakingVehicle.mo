within BobLib.Experiments.Standards.Examples;
model SpatialBrakingVehicle "Spatial mounts with selectable brake reaction locations"

  extends BobLib.Icons.SimulationIcon;
  parameter BobLib.Brakes.Types.Placement frontBrakePlacement = BobLib.Brakes.Types.Placement.Outboard
    "Front brake location selecting upright or sprung-frame reaction paths" annotation(Dialog(group = "Brakes"), choices(
      choice = BobLib.Brakes.Types.Placement.Outboard "Outboard",
      choice = BobLib.Brakes.Types.Placement.Inboard "Inboard"));
  parameter BobLib.Brakes.Types.Placement rearBrakePlacement = BobLib.Brakes.Types.Placement.DifferentialInput
    "Rear brake location selecting wheel, halfshaft or differential-input braking" annotation(Dialog(group = "Brakes"));
  parameter Real differentialInputRatio(min = Modelica.Constants.small) = 2
    "Differential-input speed divided by wheel speed" annotation(Dialog(group = "Brakes"));

  BobLib.Experiments.Standards.VehicleFMI vehicle(
    includeRotorEffects = true, initialVel = 15,
    frontBrakePlacement = frontBrakePlacement, rearBrakePlacement = rearBrakePlacement,
    pVehicle(driveline(inputRatio = differentialInputRatio))) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.Constant steering(k = 0) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant accelerator(k = 0) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Step brake(height = 0.3, startTime = 0.25) annotation(
    Placement(transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(steering.y, vehicle.steeringAngleCommand) annotation(Line(points = {{-49, 40}, {-20, 40}, {-20, 14}, {10, 14}}, color = {0, 0, 127}));
  connect(accelerator.y, vehicle.acceleratorPedalCommand) annotation(Line(points = {{-49, 0}, {-30, 0}, {-30, 9}, {10, 9}}, color = {0, 0, 127}));
  connect(brake.y, vehicle.brakePedalCommand) annotation(Line(points = {{-49, -40}, {-10, -40}, {-10, 4}, {10, 4}}, color = {0, 0, 127}));
  annotation(experiment(StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})),
    Documentation(info = "<html><p>Explicit command sources drive the same plant boundary used by the FMU. Outboard front brakes and one rear differential-input brake demonstrate distinct rotating and housing reaction paths. Choose Outboard or Inboard for either axle, or DifferentialInput for the rear. These selections change topology and require recompilation. The differential input ratio is downstream of the separate transmission.</p><p>Open vehicle to inspect its VI subsystem and mounting diagram. Motor, gearbox and caliper supports are always active; this example also enables rotor effects; full-system mass properties already include locked rotor inertia. FourPostSim instead uses prescribed-wheel static loading and an ideal differential split, not dynamic LSD behavior.</p></html>"));
end SpatialBrakingVehicle;
