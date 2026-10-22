within BobLib.Experiments.Standards.BaseClasses;
partial model PartialVehicle "Powertrain-neutral subsystem slots; derived assemblies supply implementations and wheel connections"

  replaceable BobLib.Chassis.Interfaces.Base chassis annotation(choicesAllMatching = true,
    Placement(transformation(origin = {90, -50}, extent = {{-32, -20}, {32, 20}})))
    constrainedby BobLib.Chassis.Interfaces.Base;
  replaceable BobLib.Drivelines.Interfaces.Base driveline annotation(choicesAllMatching = true,
    Placement(transformation(origin = {25, -50}, extent = {{-15, -15}, {15, 15}})))
    constrainedby BobLib.Drivelines.Interfaces.Base;
  replaceable BobLib.Brakes.Interfaces.Base brakes annotation(choicesAllMatching = true,
    Placement(transformation(origin = {155, -49}, extent = {{-15, -15}, {15, 15}})))
    constrainedby BobLib.Brakes.Interfaces.Base;
  replaceable BobLib.DriverEnvironments.Interfaces.Base driverEnvironment annotation(choicesAllMatching = true,
    Placement(transformation(origin = {40, 60}, extent = {{-20, -20}, {20, 20}})))
    constrainedby BobLib.DriverEnvironments.Interfaces.Base;
  inner replaceable VehicleInterfaces.Roads.FlatRoad road annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-60, -150}, extent = {{-20, -10}, {20, 10}})))
    constrainedby VehicleInterfaces.Roads.Interfaces.Base;
  inner replaceable VehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-120, -150}, extent = {{-20, -10}, {20, 10}})))
    constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base;

  inner replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    n = {0, 0, -1},
    driveTrainMechanics3D = false) annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-190, -170}, extent = {{-10, -10}, {10, 10}})))
    constrainedby Modelica.Mechanics.MultiBody.World;

protected
  constant Boolean showSteeringConnection = false "Show steering route in assemblies with physical steering ports";
  BobLib.Utilities.Interfaces.ControlBus controlBus
    "Internal VehicleInterfaces control bus namespace" annotation(
    Placement(
      transformation(origin = {-200, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
      iconTransformation(origin = {-180, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

equation
  connect(controlBus, chassis.controlBus) annotation(
    Line(points = {{-200, 30}, {49, 30}, {49, -38}, {58, -38}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, driveline.controlBus) annotation(
    Line(points = {{-200, 30}, {1, 30}, {1, -41}, {10, -41}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(
    Line(points = {{-200, 30}, {131, 30}, {131, -40}, {140, -40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, driverEnvironment.controlBus) annotation(
    Line(points = {{-200, 30}, {100, 30}, {100, 72}, {60, 72}}, color = {255, 204, 51}, thickness = 0.5));
  connect(driverEnvironment.steeringWheel, chassis.steeringWheel) annotation(
    Line(visible = showSteeringConnection, points = {{60, 60}, {60, 60.5}, {90, 60.5}, {90, -30}}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -180}, {180, 120}})),
    Documentation(info = "<html><p>Shared VI subsystem declarations, control bus and steering connections. Assemblies with physical pedals own their pedal connections. Derived assemblies supply the powertrain, wheel/shaft topology, mounting paths, calibration, and measurements. No BobLib-specific connectors or EV components are required by this boundary.</p></html>"));
end PartialVehicle;
