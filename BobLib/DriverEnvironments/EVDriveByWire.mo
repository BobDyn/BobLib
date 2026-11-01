within BobLib.DriverEnvironments;

model EVDriveByWire

  "BobLib EV driver-environment adapter with explicit EV direct-command pins"
  extends BobLib.DriverEnvironments.Internal.Driver;

  Modelica.Blocks.Interfaces.RealInput motorTorqueCommand(
    quantity = "Torque",
    unit = "N.m") "Requested motor torque command" annotation(
      Placement(
        transformation(origin = {-140, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90),
        iconTransformation(origin = {-80, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Blocks.Interfaces.RealInput regenTorqueLimitCommand(
    quantity = "Torque",
    unit = "N.m") "Regenerative torque limit command" annotation(
      Placement(
        transformation(origin = {-40, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90),
        iconTransformation(origin = {-50, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Blocks.Interfaces.BooleanInput inverterEnableCommand
    "Inverter enable command" annotation(
      Placement(
        transformation(origin = {-180, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -90),
        iconTransformation(origin = {-20, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

protected
  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus
    "Typed electricMotorControlBus signal contract" annotation(
      Placement(transformation(origin = {80, 90}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(electricMotorControlBus, controlBus.electricMotorControlBus) annotation(
    Line(points = {{80, 90}, {100, 90}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(motorTorqueCommand, electricMotorControlBus.driverTorqueCommand) annotation(
    Line(points = {{-140, 100}, {-140, 80}, {80, 80}, {80, 90}}, color = {0, 0, 127}));
  connect(regenTorqueLimitCommand, electricMotorControlBus.driverRegenTorqueLimit) annotation(
    Line(points = {{-40, 100}, {-40, 90}, {80, 90}}, color = {0, 0, 127}));
  connect(inverterEnableCommand, driverBus.inverterEnable) annotation(
    Line(points = {{-180, 100}, {-180, 70}, {60, 70}, {60, 60}}, color = {255, 0, 255}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>EVDriveByWire</code> extends
<code>DriverEnvironments.Internal.Driver</code> with EV-specific driver/R2D
signals. It does not include automatic/manual transmission integer commands.
</p>
<p>
The EV-specific torque and regen signals represent direct driver intent. VCU
speed-control modes may leave those inputs inactive, subscribe to the driver
and plant measurement buses, and publish their own electric-drive and mechanical
brake actuator requests on the appropriate control sub-buses.
</p>
</html>"),
    Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true)));
end EVDriveByWire;
