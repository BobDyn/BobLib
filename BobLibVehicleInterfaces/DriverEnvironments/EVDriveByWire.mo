within BobLibVehicleInterfaces.DriverEnvironments;
model EVDriveByWire
  "Optional BobLib EV driver-environment adapter with VCU command passthrough"
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

  Modelica.Blocks.Interfaces.RealOutput cmd_torque_motor(
    quantity = "Torque",
    unit = "N.m") "Motor torque command for the VCU" annotation(
      Placement(transformation(origin = {120, -116}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput cmd_regen_limit(
    quantity = "Torque",
    unit = "N.m") "Regenerative torque limit command for the VCU" annotation(
      Placement(transformation(origin = {120, -148}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.BooleanOutput cmd_inverter_enable
    "Inverter enable command for the VCU" annotation(
      Placement(transformation(origin = {120, -180}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(motorTorqueCommand, cmd_torque_motor) annotation(
    Line(points = {{-120, -116}, {120, -116}}, color = {0, 0, 127}));
  connect(regenTorqueLimitCommand, cmd_regen_limit) annotation(
    Line(points = {{-120, -148}, {120, -148}}, color = {0, 0, 127}));
  connect(inverterEnableCommand, cmd_inverter_enable) annotation(
    Line(points = {{-120, -180}, {120, -180}}, color = {255, 0, 255}));

  annotation(Documentation(info = "<html>
<p>
Model <code>EVDriveByWire</code> extends
<code>AutomaticDriveByWire</code> with EV-specific command passthroughs for
motor torque, regenerative torque limit, and inverter enable.
</p>
<p>
It remains an optional adapter. Standard BobLib vehicle simulations send driver
commands directly to the VCU and publish brake/driver intent on the
VehicleInterfaces driver bus; this model is useful when a derived architecture
wants those signals to cross an explicit driver-environment boundary.
</p>
</html>"));
end EVDriveByWire;
