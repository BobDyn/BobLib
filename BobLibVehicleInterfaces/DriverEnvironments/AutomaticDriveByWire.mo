within BobLibVehicleInterfaces.DriverEnvironments;
model AutomaticDriveByWire
  "Optional BobLib automatic driver-environment adapter"
  extends VehicleInterfaces.Icons.DriverEnvironment;
  extends VehicleInterfaces.DriverEnvironments.Interfaces.BaseAutomaticTransmission(
    final includeDriverSeat = true,
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
  VehicleInterfaces.Interfaces.DriverBus driverBus annotation(
    Placement(transformation(extent = {{48, 56}, {68, 76}})));

  Modelica.Mechanics.Rotational.Sources.Position steeringPosition(
    exact = true,
    w(start = 0)) annotation(
      Placement(transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(controlBus.driverBus, driverBus) annotation(
    Line(points = {{100, 60}, {76, 60}, {76, 66}, {58, 66}}, color = {255, 204, 51}, thickness = 0.5));
  connect(steeringAngleCommand, steeringPosition.phi_ref) annotation(
    Line(points = {{-120, 80}, {26, 80}, {26, 0}, {46, 0}}, color = {0, 0, 127}));
  connect(steeringPosition.flange, steeringWheel) annotation(
    Line(points = {{68, 0}, {100, 0}}));
  connect(acceleratorPedalCommand, driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-120, 48}, {58, 48}, {58, 66}}, color = {0, 0, 127}));
  connect(brakePedalCommand, driverBus.brakePedalPosition) annotation(
    Line(points = {{-120, 16}, {56, 16}, {56, 66}, {58, 66}}, color = {0, 0, 127}));
  connect(requestedGearCommand, driverBus.requestedGear) annotation(
    Line(points = {{-120, -20}, {62, -20}, {62, 66}}, color = {255, 127, 0}));
  connect(gearboxModeCommand, driverBus.gearboxMode) annotation(
    Line(points = {{-120, -52}, {64, -52}, {64, 66}}, color = {255, 127, 0}));
  connect(ignitionCommand, driverBus.ignition) annotation(
    Line(points = {{-120, -84}, {66, -84}, {66, 66}}, color = {255, 127, 0}));

  annotation(Documentation(info = "<html>
<p>
Model <code>AutomaticDriveByWire</code> is an optional BobLib
driver-environment adapter. It does not define maneuver behavior itself.
Instead, it receives external steering, accelerator, brake, gear, gearbox-mode,
and ignition commands, publishes the standard VehicleInterfaces
<code>driverBus</code> signals, and drives the standard steering-wheel flange.
</p>
<p>
Full-vehicle BobLib standard simulations are autonomous by default and can wire
their command sources directly to the VCU and driver bus. Insert this adapter in
a derived vehicle only when an explicit driver-environment block is useful for
the diagram or for coupling an external driver model.
</p>
</html>"));
end AutomaticDriveByWire;
