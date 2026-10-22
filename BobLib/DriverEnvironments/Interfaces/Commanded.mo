within BobLib.DriverEnvironments.Interfaces;
partial model Commanded "VI driver environment with radian steering and normalized pedal commands"

  extends BobLib.DriverEnvironments.Interfaces.Base(
    final includeDriverSeat = false,
    final includeSteeringWheel = true,
    final includeAcceleratorPedal = false,
    final includeBrakePedal = false);

  parameter Boolean exactSteering = true
    "Use analytical command derivatives; disable for externally sampled commands"
    annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Modelica.Units.SI.Frequency steeringFilterFrequency = 50
    "MSL steering position filter cutoff when exactSteering=false" annotation(Dialog(group = "Operating settings"));

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad") "Commanded steering wheel angle" annotation(
    Placement(
      transformation(origin = {-200, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0),
      iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput acceleratorPedalCommand(unit = "1")
    "Commanded accelerator pedal position" annotation(
    Placement(
      transformation(origin = {-200, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0),
      iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput brakePedalCommand(unit = "1")
    "Commanded brake pedal position" annotation(
    Placement(
      transformation(origin = {-200, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0),
      iconTransformation(origin = {-110, 20}, extent = {{-10, -10}, {10, 10}})));

  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true)));
end Commanded;
