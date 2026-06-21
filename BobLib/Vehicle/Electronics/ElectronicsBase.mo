within BobLib.Vehicle.Electronics;

partial model ElectronicsBase

  "Vehicle-level electronics interface contract"
  extends BobLib.Resources.Icons.ElectronicsBaseIcon;

  import SI = Modelica.Units.SI;

  Modelica.Electrical.Analog.Interfaces.PositivePin p
    "HV bus positive terminal" annotation(
      Placement(transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin n
    "HV bus negative terminal" annotation(
      Placement(transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput cmd_torque_motor
    "Requested motor torque [Nm]" annotation(
      Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput cmd_regen_limit
    "Maximum regen torque magnitude [Nm]" annotation(
      Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.BooleanInput cmd_inverter_enable
    "Inverter enable / R2D" annotation(
      Placement(transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealInput sens_motor_speed
    "Motor speed [rad/s]" annotation(
      Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

  Modelica.Blocks.Interfaces.RealOutput P_motor
    "Electrical power delivered to the motor side [W]" annotation(Placement(transformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}})),
    Icon(graphics = {
      Line(points = {{-96, 40}, {-78, 40}}, color = {0, 0, 255}),
      Line(points = {{-96, -40}, {-78, -40}}, color = {0, 0, 255}),
      Line(points = {{78, 0}, {100, 0}}, color = {0, 0, 127})
    }));

  parameter SI.Torque tau_max = 220
    "VCU motoring torque limit";
  parameter SI.AngularVelocity w_eps = 1.0
    "VCU low-speed regularization";
  parameter Real motorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed";
  parameter SI.Power inverterPMaxMot = 124e3
    "Inverter maximum motoring output power";
  parameter SI.Power inverterPMaxReg = 124e3
    "Inverter maximum regenerative input power magnitude";
  parameter SI.Voltage inverterVdcMax = 588
    "Inverter regen cutoff voltage";

  output SI.Voltage V_dc "HV bus voltage";
  output SI.Current I_dc "HV bus current, positive while discharging";
  output SI.Power P_req "VCU power request";
  output SI.Torque tau_cmd_limited "VCU limited torque command";
  output Boolean active "Electronics enable state";

end ElectronicsBase;
