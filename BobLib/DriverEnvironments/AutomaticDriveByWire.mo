within BobLib.DriverEnvironments;

model AutomaticDriveByWire "BobLib automatic driver-environment adapter"

  extends BobLib.DriverEnvironments.Internal.Driver;

  Modelica.Blocks.Interfaces.IntegerInput gearboxModeCommand
    "Gearbox mode command" annotation(
    Placement(
      transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.IntegerInput ignitionCommand
    "Ignition command" annotation(
    Placement(
      transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(gearboxModeCommand, controlBus.driverBus.gearboxMode) annotation(
    Line(points = {{-120, -40}, {20, -40}, {20, 60}, {100, 60}}, color = {255, 127, 0}));
  connect(ignitionCommand, controlBus.driverBus.ignition) annotation(
    Line(points = {{-120, -80}, {20, -80}, {20, 60}, {100, 60}}, color = {255, 127, 0}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>AutomaticDriveByWire</code> is the public BobLib adapter for an
explicit automatic-transmission driver-environment boundary. It composes the
minimal internal <code>DriverEnvironments.Internal.Driver</code> publisher,
which receives external steering, accelerator, and brake commands. This adapter
adds automatic-transmission gearbox mode and ignition inputs and publishes them
onto the standard VehicleInterfaces <code>driverBus</code>. Direct gear
selection belongs in a manual/transmission-specific adapter.
</p>
<p>
Architecture-specific driver signals, such as EV ready-to-drive or direct motor
commands, belong in specialized adapters or vehicle templates that explicitly
need those pins.
</p>
</html>"));
end AutomaticDriveByWire;
