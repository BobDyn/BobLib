within BobLibVehicleInterfaces.Controllers.Internal;

model VCUCore
  import SI = Modelica.Units.SI;

  /**********************
   * Inputs (Commands)
   **********************/
  Modelica.Blocks.Interfaces.RealInput cmd_torque_motor
    "Requested motor torque [Nm]" annotation(
      Placement(transformation(origin={-120,40}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_regen_limit
    "Max allowed regen torque (positive magnitude) [Nm]" annotation(
      Placement(transformation(origin={-120,-40}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.BooleanInput cmd_inverter_enable
    "Inverter enable / R2D" annotation(
      Placement(transformation(origin={-120,0}, extent={{-20,-20},{20,20}})));

  /**********************
   * Inputs (Sensors)
   **********************/
  Modelica.Blocks.Interfaces.RealInput sens_motor_speed
    "Motor speed [rad/s]" annotation(
      Placement(transformation(origin={-120,80}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.RealInput sens_hv_bus_voltage
    "HV bus voltage [V]" annotation(
      Placement(transformation(origin={-120,-80}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.RealInput sens_hv_bus_current
    "HV bus current [A]" annotation(
      Placement(transformation(origin={-120,-120}, extent={{-20,-20},{20,20}})));

  /**********************
   * Outputs
   **********************/
  Modelica.Blocks.Interfaces.RealOutput P_req
    "Power request to inverter [W]" annotation(
      Placement(transformation(origin={120,0}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.RealOutput tau_cmd_limited
    "Final torque command after limits [Nm]" annotation(
      Placement(transformation(origin={120,40}, extent={{-20,-20},{20,20}})));

  Modelica.Blocks.Interfaces.BooleanOutput vcu_active
    "VCU active / inverter enabled" annotation(
      Placement(transformation(origin={120,-40}, extent={{-20,-20},{20,20}})));

  /**********************
   * Parameters
   **********************/
  parameter SI.Torque tau_max = 240
    "Max motoring torque [Nm]";

  parameter SI.AngularVelocity w_eps = 1e-2
    "Small speed for launch protection";
  parameter Real motorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed";

protected
  SI.Torque tau_cmd_raw;
  SI.AngularVelocity sens_motor_speed_drive;
  SI.AngularVelocity w_eff;

equation
  // Safety / enable
  vcu_active = cmd_inverter_enable;

  // Raw torque command
  tau_cmd_raw =
    if not cmd_inverter_enable then
      0
    else
      min(
        max(cmd_torque_motor, -cmd_regen_limit),
        tau_max
      );

  tau_cmd_limited = tau_cmd_raw;

  // Effective speed (avoid zero divide)
  sens_motor_speed_drive = motorSpeedSign*sens_motor_speed;
  w_eff =
    if noEvent(abs(sens_motor_speed_drive) > w_eps) then
      sens_motor_speed_drive
    elseif noEvent(tau_cmd_limited >= 0) then
      w_eps
    else
      -w_eps;

  // Torque -> Power
  P_req = tau_cmd_limited * w_eff;

  annotation(
    Documentation(info = "<html>
<p>
Model <code>VCUCore</code> implements the BobLib VCU torque and power request logic.
</p>
<p>
It limits motoring and regenerative torque, handles inverter enable state, protects launch behavior near zero speed, and outputs the power request consumed by the inverter path.
</p>
</html>"));
end VCUCore;
