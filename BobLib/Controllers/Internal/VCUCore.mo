within BobLib.Controllers.Internal;

model VCUCore

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Controllers.VCURecord pVCU(tau_max = 240) "Torque limits, reduction and brake blending calibration" annotation(Dialog(group = "Calibration"));

  // Inputs (Commands)
  Modelica.Blocks.Interfaces.RealInput cmd_torque_motor
    "Requested motor torque [Nm]" annotation(
      Placement(
        transformation(origin = {120, 80}, extent = {{-8, -8}, {8, 8}}, rotation = -90),
        iconTransformation(origin = {120, 80}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));

  Modelica.Blocks.Interfaces.RealInput cmd_regen_limit

    "Max allowed regen torque (positive magnitude) [Nm]" annotation(
      Placement(
        transformation(origin = {-160, 30}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {-160, 30}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  Modelica.Blocks.Interfaces.BooleanInput cmd_inverter_enable
    "Inverter enable / R2D" annotation(
      Placement(
        transformation(origin = {-160, 60}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {-160, 60}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  // Inputs (Sensors)
  Modelica.Blocks.Interfaces.RealInput sens_motor_speed
    "Motor speed [rad/s]" annotation(
      Placement(
        transformation(origin = {-160, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {-160, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  // Outputs
  Modelica.Blocks.Interfaces.RealOutput P_req
    "Power request to inverter [W]" annotation(
      Placement(
        transformation(origin = {160, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {160, 0}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealOutput tau_cmd_limited
    "Final torque command after limits [Nm]" annotation(
      Placement(
        transformation(origin = {160, 50}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {160, 50}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  Modelica.Blocks.Interfaces.BooleanOutput vcu_active
    "VCU active / inverter enabled" annotation(
      Placement(
        transformation(origin = {160, -50}, extent = {{-8, -8}, {8, 8}}, rotation = 0),
        iconTransformation(origin = {160, -50}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  // Internal variables

protected
  SI.Torque tau_motoring_limit;
  SI.AngularVelocity sens_motor_speed_drive;
  SI.AngularVelocity w_eff;
  discrete Real speedDirection(start = 1, fixed = true)
    "Established drive-positive rotation direction, held inside the launch deadband";

equation

  // Safety / enable
  vcu_active = cmd_inverter_enable;

  sens_motor_speed_drive = pVCU.motorSpeedSign*sens_motor_speed;
  tau_motoring_limit = min(
    pVCU.tau_max,
    pVCU.P_max_mot/max(abs(sens_motor_speed_drive), pVCU.w_eps));

  // Raw torque command
  tau_cmd_limited =

    if not cmd_inverter_enable then
      0
    else
      min(
        max(cmd_torque_motor, -cmd_regen_limit),
        tau_motoring_limit
      );

  // Match the motor's direction hysteresis so rollback cannot flip torque at -w_eps.
  when initial() then
    speedDirection = if abs(sens_motor_speed_drive) > pVCU.w_eps then
      (if sens_motor_speed_drive > 0 then 1 else -1) else 1;
  elsewhen sens_motor_speed_drive > pVCU.w_eps then
    speedDirection = 1;
  elsewhen sens_motor_speed_drive < -pVCU.w_eps then
    speedDirection = -1;
  end when;
  w_eff = speedDirection*noEvent(max(abs(sens_motor_speed_drive), pVCU.w_eps));

  // Torque -> Power
  P_req = tau_cmd_limited * w_eff;

  annotation(
    Documentation(info = "<html>
<p>
Model <code>VCUCore</code> implements the BobLib VCU torque and power request logic.
</p>
<p>
It limits motoring and regenerative torque, handles inverter enable state, protects launch behavior near zero speed, and outputs the power request consumed by the inverter path.
The core accepts motor torque demand, a regenerative torque limit, inverter
enable, and measured motor speed. The public VCU adapter converts pedal intent
into these requests and owns regenerative/mechanical brake blending.
</p>
<p>
The speed direction is held inside the launch deadband. When used with the
BobLib power-controlled motor, use the same <code>w_eps</code> for the VCU and
motor so their torque-to-power and power-to-torque direction transitions agree.
The vehicle definition records supply matching defaults.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-160, -80}, {160, 80}}, preserveAspectRatio = true)));
  annotation(Icon(coordinateSystem(extent = {{-160, -80}, {160, 80}}), graphics = {Rectangle(extent = {{-160, 80}, {160, -80}}, fillColor = {240, 240, 240}, fillPattern = FillPattern.Solid), Text(extent = {{-140, 25}, {140, -25}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0)}));
end VCUCore;
