within BobLibVehicleInterfaces.DriverEnvironments;
model EVDriveByWire
  "BobLib EV driver-environment adapter with bus command publishing"
  extends BobLibVehicleInterfaces.DriverEnvironments.AutomaticDriveByWire;

  Modelica.Blocks.Interfaces.RealInput motorTorqueCommand(
    quantity = "Torque",
    unit = "N.m") "Requested motor torque command" annotation(
      Placement(transformation(origin = {-120, -116}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput regenTorqueLimitCommand(
    quantity = "Torque",
    unit = "N.m") "Regenerative torque limit command" annotation(
      Placement(transformation(origin = {-120, -148}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.BooleanInput inverterEnableCommand
    "Inverter enable command" annotation(
      Placement(transformation(origin = {-120, -180}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(motorTorqueCommand, controlBus.electricMotorControlBus.driverTorqueCommand) annotation(
    Line(points = {{-120, -116}, {100, -116}, {100, 60}}, color = {0, 0, 127}));
  connect(regenTorqueLimitCommand, controlBus.electricMotorControlBus.driverRegenTorqueLimit) annotation(
    Line(points = {{-120, -148}, {100, -148}, {100, 60}}, color = {0, 0, 127}));
  connect(inverterEnableCommand, controlBus.driverBus.inverterEnable) annotation(
    Line(points = {{-120, -180}, {100, -180}, {100, 60}}, color = {255, 0, 255}));

  annotation(Documentation(info = "<html>
<p>
Model <code>EVDriveByWire</code> extends
<code>AutomaticDriveByWire</code> with EV-specific bus publishers for motor
torque, regenerative torque limit, and inverter enable.
</p>
<p>
The EV-specific torque and regen signals represent direct driver intent. VCU
speed-control modes may leave those inputs inactive, subscribe to the driver
and plant measurement buses, and publish their own electric-drive and mechanical
brake actuator requests on the appropriate control sub-buses.
</p>
</html>"));
end EVDriveByWire;
