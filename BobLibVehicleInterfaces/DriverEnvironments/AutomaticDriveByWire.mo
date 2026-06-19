within BobLibVehicleInterfaces.DriverEnvironments;
model AutomaticDriveByWire
  "BobLib automatic driver-environment adapter"
  extends VehicleInterfaces.Icons.DriverEnvironment;
  extends VehicleInterfaces.DriverEnvironments.Interfaces.BaseAutomaticTransmission(
    final includeDriverSeat = false,
    final includeSteeringWheel = true,
    final includeAcceleratorPedal = false,
    final includeBrakePedal = false);

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad") "Commanded steering wheel angle" annotation(
      Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput acceleratorPedalCommand(
    unit = "1") "Commanded accelerator pedal position" annotation(
      Placement(transformation(origin = {-120, 48}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput brakePedalCommand(
    unit = "1") "Commanded brake pedal position" annotation(
      Placement(transformation(origin = {-120, 16}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput requestedGearCommand
    "Requested gear command" annotation(
      Placement(transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput gearboxModeCommand
    "Gearbox mode command" annotation(
      Placement(transformation(origin = {-120, -52}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput ignitionCommand
    "Ignition command" annotation(
      Placement(transformation(origin = {-120, -84}, extent = {{-20, -20}, {20, 20}})));

protected
  Modelica.Mechanics.Rotational.Sources.Position steeringPosition(
    exact = true,
    w(start = 0)) annotation(
      Placement(transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(steeringAngleCommand, steeringPosition.phi_ref) annotation(
    Line(points = {{-120, 80}, {26, 80}, {26, 0}, {46, 0}}, color = {0, 0, 127}));
  connect(steeringPosition.flange, steeringWheel) annotation(
    Line(points = {{68, 0}, {100, 0}}));
  connect(steeringAngleCommand, controlBus.driverBus.steeringWheelAngle) annotation(
    Line(points = {{-120, 80}, {100, 80}, {100, 60}}, color = {0, 0, 127}));
  connect(acceleratorPedalCommand, controlBus.driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-120, 48}, {100, 48}, {100, 60}}, color = {0, 0, 127}));
  connect(brakePedalCommand, controlBus.driverBus.brakePedalPosition) annotation(
    Line(points = {{-120, 16}, {100, 16}, {100, 60}}, color = {0, 0, 127}));
  connect(requestedGearCommand, controlBus.driverBus.requestedGear) annotation(
    Line(points = {{-120, -20}, {100, -20}, {100, 60}}, color = {255, 127, 0}));
  connect(gearboxModeCommand, controlBus.driverBus.gearboxMode) annotation(
    Line(points = {{-120, -52}, {100, -52}, {100, 60}}, color = {255, 127, 0}));
  connect(ignitionCommand, controlBus.driverBus.ignition) annotation(
    Line(points = {{-120, -84}, {100, -84}, {100, 60}}, color = {255, 127, 0}));

  annotation(Documentation(info = "<html>
<p>
Model <code>AutomaticDriveByWire</code> is the BobLib adapter for an explicit
driver-environment boundary. It does not define maneuver behavior itself.
Instead, it receives external steering, accelerator, brake, gear, gearbox-mode,
and ignition commands, publishes the standard VehicleInterfaces
<code>driverBus</code> signals, and drives the standard steering-wheel flange.
The steering command is also published as
<code>controlBus.driverBus.steeringWheelAngle</code> for controller
subscribers.
</p>
<p>
Full-vehicle BobLib standard simulations are autonomous by default, but their
maneuver sources still cross this driver-environment boundary before entering
the shared <code>driverBus</code>. Derived vehicles can replace the command
sources, this adapter, or both when coupling an external driver model.
</p>
</html>"));
end AutomaticDriveByWire;
