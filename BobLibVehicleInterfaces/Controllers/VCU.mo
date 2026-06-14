within BobLibVehicleInterfaces.Controllers;
model VCU
  "BobLib VCU exposed inside the VehicleInterfaces controller domain"
  extends VehicleInterfaces.Controllers.Interfaces.Base;

  import SI = Modelica.Units.SI;

  Modelica.Blocks.Interfaces.RealInput cmd_torque_motor
    "Requested motor torque [Nm]" annotation(
      Placement(transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_steering_angle(
    quantity = "Angle",
    unit = "rad")
    "Driver steering-wheel command [rad]" annotation(
      Placement(transformation(origin = {-120, 120}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_accelerator_pedal(
    unit = "1")
    "Driver accelerator-pedal command" annotation(
      Placement(transformation(origin = {-120, 96}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_brake_pedal(
    unit = "1")
    "Driver brake-pedal command for regen blending and brake coordination" annotation(
      Placement(transformation(origin = {-120, 72}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_regen_limit
    "Max allowed regen torque magnitude [Nm]" annotation(
      Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.BooleanInput cmd_inverter_enable
    "Inverter enable / R2D" annotation(
      Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput cmd_requested_gear
    "Driver requested gear" annotation(
      Placement(transformation(origin = {-120, -64}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput cmd_gearbox_mode
    "Driver requested gearbox mode" annotation(
      Placement(transformation(origin = {-120, -88}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.IntegerInput cmd_ignition
    "Driver ignition command" annotation(
      Placement(transformation(origin = {-120, -112}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput sens_motor_speed
    "Motor speed [rad/s]" annotation(
      Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput sens_hv_bus_voltage
    "HV bus voltage [V]" annotation(
      Placement(transformation(origin = {-120, -144}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput sens_hv_bus_current
    "HV bus current [A]" annotation(
      Placement(transformation(origin = {-120, -168}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput P_req
    "Power request to inverter [W]" annotation(
      Placement(transformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput tau_cmd_limited
    "Final torque command after limits [Nm]" annotation(
      Placement(transformation(origin = {120, 40}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.BooleanOutput vcu_active
    "VCU active / inverter enabled" annotation(
      Placement(transformation(origin = {120, -40}, extent = {{-20, -20}, {20, 20}})));

  parameter SI.Torque tau_max = 240
    "Max motoring torque [Nm]";
  parameter SI.AngularVelocity w_eps = 1e-2
    "Small speed for launch protection";
  parameter Real motorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed";

protected
  BobLibVehicleInterfaces.Controllers.Internal.VCUCore vcu(
    tau_max = tau_max,
    w_eps = w_eps,
    motorSpeedSign = motorSpeedSign) annotation(
      Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(cmd_torque_motor, vcu.cmd_torque_motor) annotation(
    Line(points = {{-120, 40}, {-24, 40}, {-24, 8}}, color = {0, 0, 127}));
  connect(cmd_steering_angle, vcu.cmd_steering_angle) annotation(
    Line(points = {{-120, 120}, {-34, 120}, {-34, 12}, {-24, 12}}, color = {0, 0, 127}));
  connect(cmd_accelerator_pedal, vcu.cmd_accelerator_pedal) annotation(
    Line(points = {{-120, 96}, {-38, 96}, {-38, 9.6}, {-24, 9.6}}, color = {0, 0, 127}));
  connect(cmd_brake_pedal, vcu.cmd_brake_pedal) annotation(
    Line(points = {{-120, 72}, {-42, 72}, {-42, 7.2}, {-24, 7.2}}, color = {0, 0, 127}));
  connect(cmd_regen_limit, vcu.cmd_regen_limit) annotation(
    Line(points = {{-120, -40}, {-24, -40}, {-24, -8}}, color = {0, 0, 127}));
  connect(cmd_inverter_enable, vcu.cmd_inverter_enable) annotation(
    Line(points = {{-120, 0}, {-24, 0}}, color = {255, 0, 255}));
  connect(cmd_requested_gear, vcu.cmd_requested_gear) annotation(
    Line(points = {{-120, -64}, {-44, -64}, {-44, -12.8}, {-24, -12.8}}, color = {255, 127, 0}));
  connect(cmd_gearbox_mode, vcu.cmd_gearbox_mode) annotation(
    Line(points = {{-120, -88}, {-40, -88}, {-40, -17.6}, {-24, -17.6}}, color = {255, 127, 0}));
  connect(cmd_ignition, vcu.cmd_ignition) annotation(
    Line(points = {{-120, -112}, {-36, -112}, {-36, -22.4}, {-24, -22.4}}, color = {255, 127, 0}));
  connect(sens_motor_speed, vcu.sens_motor_speed) annotation(
    Line(points = {{-120, 80}, {-24, 80}, {-24, 16}}, color = {0, 0, 127}));
  connect(sens_hv_bus_voltage, vcu.sens_hv_bus_voltage) annotation(
    Line(points = {{-120, -144}, {-32, -144}, {-32, -16}, {-24, -16}}, color = {0, 0, 127}));
  connect(sens_hv_bus_current, vcu.sens_hv_bus_current) annotation(
    Line(points = {{-120, -168}, {-28, -168}, {-28, -24}, {-24, -24}}, color = {0, 0, 127}));
  connect(vcu.P_req, P_req) annotation(
    Line(points = {{24, 0}, {120, 0}}, color = {0, 0, 127}));
  connect(vcu.tau_cmd_limited, tau_cmd_limited) annotation(
    Line(points = {{24, 8}, {80, 8}, {80, 40}, {120, 40}}, color = {0, 0, 127}));
  connect(vcu.vcu_active, vcu_active) annotation(
    Line(points = {{24, -8}, {80, -8}, {80, -40}, {120, -40}}, color = {255, 0, 255}));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>VCU</code> is the public controller adapter exposed inside the VehicleInterfaces controller domain.
</p>
<p>
It accepts driver-level steering, accelerator, brake, gear, gearbox-mode, and
ignition commands alongside actuator-level motor-torque, regen-limit, enable,
motor-speed, and HV-bus feedback signals. The brake command is intentionally
visible here so future VCU logic can own regenerative-braking blending while
the same command remains available to the mechanical brake subsystem through
the VehicleInterfaces driver bus.
</p>
</html>"));
end VCU;
