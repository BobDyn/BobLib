within BobLib.DriverEnvironments;

model AutomaticDriveByWire "BobLib automatic driver-environment adapter"

  extends BobLib.DriverEnvironments.Internal.Driver;

  parameter VehicleInterfaces.Types.Gear manualGearRequest = 0
    "Requested gear in manual mode; zero when no direct selection is requested" annotation(Dialog(group = "Operating settings"));

  Modelica.Blocks.Sources.IntegerConstant requestedGear(k = manualGearRequest) annotation(
    Placement(transformation(origin = {-150, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.IntegerInput gearboxModeCommand
    "Gearbox mode command" annotation(
    Placement(
      transformation(origin = {-110, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90),
      iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.IntegerInput ignitionCommand
    "Ignition command" annotation(
    Placement(
      transformation(origin = {10, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90),
      iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(requestedGear.y, driverBus.requestedGear) annotation(
    Line(points = {{-139, 80}, {-130, 80}, {-130, 70}, {60, 70}, {60, 60}}, color = {255, 127, 0}));
  connect(gearboxModeCommand, driverBus.gearboxMode) annotation(
    Line(points = {{-110, 100}, {-110, 80}, {60, 80}, {60, 60}}, color = {255, 127, 0}));
  connect(ignitionCommand, driverBus.ignition) annotation(
    Line(points = {{10, 100}, {10, 90}, {60, 90}, {60, 60}}, color = {255, 127, 0}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>AutomaticDriveByWire</code> is the public BobLib adapter for an
explicit automatic-transmission driver-environment boundary. It composes the
minimal internal <code>DriverEnvironments.Internal.Driver</code> publisher,
which receives external steering, accelerator, and brake commands. This adapter
adds automatic-transmission gearbox mode and ignition inputs and publishes them
onto the standard VehicleInterfaces <code>driverBus</code>. The manualGearRequest parameter supplies a fixed requested gear for manual/limited gearbox modes.
</p>
<p>
Architecture-specific driver signals, such as EV ready-to-drive or direct motor
commands, belong in specialized adapters or vehicle templates that explicitly
need those pins.
</p>
</html>"));
end AutomaticDriveByWire;
