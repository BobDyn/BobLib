within BobLibVehicleInterfaces.DriverEnvironments;

model EVDriveByWire

  "BobLib EV driver-environment adapter with explicit EV direct-command pins"
  extends BobLibVehicleInterfaces.DriverEnvironments.Internal.Driver;

  Modelica.Blocks.Interfaces.RealInput motorTorqueCommand(
    quantity = "Torque",
    unit = "N.m") "Requested motor torque command" annotation(
      Placement(
        transformation(origin = {-80, 160}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {-80, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Blocks.Interfaces.RealInput regenTorqueLimitCommand(
    quantity = "Torque",
    unit = "N.m") "Regenerative torque limit command" annotation(
      Placement(
        transformation(origin = {-40, 160}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {-50, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Blocks.Interfaces.BooleanInput inverterEnableCommand
    "Inverter enable command" annotation(
      Placement(
        transformation(origin = {40, 160}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {-20, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(motorTorqueCommand, controlBus.electricMotorControlBus.driverTorqueCommand) annotation(
    Line(points = {{-80, 160}, {-80, 130}, {100, 130}, {100, 60}}, color = {0, 0, 127}));
  connect(regenTorqueLimitCommand, controlBus.electricMotorControlBus.driverRegenTorqueLimit) annotation(
    Line(points = {{-40, 160}, {-40, 120}, {100, 120}, {100, 60}}, color = {0, 0, 127}));
  connect(inverterEnableCommand, controlBus.driverBus.inverterEnable) annotation(
    Line(points = {{40, 160}, {40, 130}, {100, 130}, {100, 60}}, color = {255, 0, 255}));

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
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 140}})));
end EVDriveByWire;
