within BobLibVehicleInterfaces.ElectricDrives;
model Motor

  "BobLib motor exposed through the VehicleInterfaces electric-drive shaft"
  extends VehicleInterfaces.Icons.ElectricMotor;
  extends VehicleInterfaces.ElectricDrives.Interfaces.Base;

  import SI = Modelica.Units.SI;

  parameter SI.Voltage Vdc_max = 630
    "Motor maximum DC voltage reference";
  parameter Real rpm_max_peak = 6500
    "Motor peak speed reference";
  parameter SI.Torque T_peak = 220
    "Motor peak torque";
  parameter SI.Torque T_cont = 130
    "Motor continuous torque";
  parameter SI.Current I_peak_2min = 360
    "Motor peak current";
  parameter SI.Current I_cont = 180
    "Motor continuous current";
  parameter Real Kt_Nm_per_A = 0.61
    "Motor torque constant";
  parameter SI.Time peakTime = 120
    "Peak torque/current allowance duration";
  parameter SI.Power P_mech_peak = 124e3
    "Motor peak mechanical power";
  parameter SI.Power P_cont_low = 75e3
    "Motor continuous power envelope low-speed anchor";
  parameter SI.Power P_cont_high = 75e3
    "Motor continuous power envelope high-speed anchor";
  parameter Real eta_mot(unit = "1") = 0.96
    "Motor motoring efficiency reference";
  parameter Real eta_reg(unit = "1") = 0.95
    "Motor regen efficiency reference";
  parameter SI.AngularVelocity w_eps = 1.0
    "Low-speed regularization for power-to-torque conversion";
  parameter SI.Inertia rotorJ = 0.02521
    "Motor rotor inertia";

  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    "Motor-side DC positive pin" annotation(
      Placement(transformation(origin = {-100, 44}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {60, 100}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    "Motor-side DC negative pin" annotation(
      Placement(transformation(origin = {-100, 20}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}})));

  BobLibVehicleInterfaces.ElectricDrives.Internal.PowerLimitedMotor motor(
    Vdc_max = Vdc_max,
    rpm_max_peak = rpm_max_peak,
    T_peak = T_peak,
    T_cont = T_cont,
    I_peak_2min = I_peak_2min,
    I_cont = I_cont,
    Kt_Nm_per_A = Kt_Nm_per_A,
    peakTime = peakTime,
    P_mech_peak = P_mech_peak,
    P_cont_low = P_cont_low,
    P_cont_high = P_cont_high,
    eta_mot = eta_mot,
    eta_reg = eta_reg,
    w_eps = w_eps) annotation(
      Placement(transformation(origin = {-20, 0}, extent = {{-30, -30}, {30, 30}})));

  Modelica.Mechanics.Rotational.Components.Inertia rotor(
    J = rotorJ) annotation(
      Placement(transformation(origin = {45, 0}, extent = {{-10, -10}, {10, 10}})));

  output SI.AngularVelocity w "Shaft speed";
  output SI.Voltage V_dc "Motor-side DC voltage";
  output SI.Current I_dc "Motor-side DC current, positive while motoring";
  output SI.Power P_elec "Electrical power absorbed by motor, positive while motoring";
  output SI.Power P_mech "Actual mechanical shaft power";
  output SI.Torque tau_cmd "Commanded motor torque";
  output SI.Torque tau_lim "Active motor torque limit";

protected
  Modelica.Blocks.Sources.RealExpression electricalPowerInput(
    y = P_elec) "Electrical power measurement fed to the internal motor model" annotation(
      Placement(transformation(origin = {-72, 6}, extent = {{-8, -5}, {8, 5}})));

  Modelica.Blocks.Sources.RealExpression speedBusSignal(
    y = w) "Shaft speed published to electricMotorBus" annotation(
      Placement(transformation(origin = {-70, -54}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression electricalPowerBusSignal(
    y = P_elec) "Electrical power published to electricMotorBus" annotation(
      Placement(transformation(origin = {-70, -64}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression mechanicalPowerBusSignal(
    y = P_mech) "Mechanical shaft power published to electricMotorBus" annotation(
      Placement(transformation(origin = {-70, -74}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression torqueCommandBusSignal(
    y = tau_cmd) "Commanded torque published to electricMotorBus" annotation(
      Placement(transformation(origin = {-70, -84}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression torqueLimitBusSignal(
    y = tau_lim) "Active torque limit published to electricMotorBus" annotation(
      Placement(transformation(origin = {-70, -94}, extent = {{-8, -4}, {8, 4}})));

equation
  0 = pin_p.i + pin_n.i;
  V_dc = pin_p.v - pin_n.v;
  I_dc = pin_p.i;
  P_elec = V_dc*I_dc;

  w = motor.w;
  P_mech = motor.P_mech;
  tau_cmd = motor.tau_cmd;
  tau_lim = motor.tau_lim;

  connect(speedBusSignal.y, controlBus.electricMotorBus.speed) annotation(
    Line(points = {{-61.2, -54}, {-88, -54}, {-88, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(electricalPowerBusSignal.y, controlBus.electricMotorBus.electricalPower) annotation(
    Line(points = {{-61.2, -64}, {-88, -64}, {-88, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(mechanicalPowerBusSignal.y, controlBus.electricMotorBus.mechanicalPower) annotation(
    Line(points = {{-61.2, -74}, {-90, -74}, {-90, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(torqueCommandBusSignal.y, controlBus.electricMotorBus.torqueCommand) annotation(
    Line(points = {{-61.2, -84}, {-92, -84}, {-92, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(torqueLimitBusSignal.y, controlBus.electricMotorBus.torqueLimit) annotation(
    Line(points = {{-61.2, -94}, {-94, -94}, {-94, -60}, {-100, -60}}, color = {0, 0, 127}));

  connect(electricalPowerInput.y, motor.P_elec) annotation(
    Line(points = {{-63.2, 6}, {-56, 6}, {-56, 0}}, color = {0, 0, 127}));
  connect(motor.shaft, rotor.flange_a) annotation(
    Line(points = {{10, 0}, {35, 0}}));
  connect(rotor.flange_b, shaft_b.flange) annotation(
    Line(points = {{55, 0}, {100, 0}}));
  annotation(Documentation(info = "<html>
<p>
Thin adapter around <code>BobLibVehicleInterfaces.ElectricDrives.Internal.PowerLimitedMotor</code>.
The mechanical side follows <code>VehicleInterfaces.ElectricDrives.Interfaces.Base</code>
and includes the motor rotor inertia so the downstream driveline can remain a
pure final-drive/differential/halfshaft assembly. The adapter publishes motor
speed, absorbed electrical power, mechanical power, commanded torque, and active
torque limit on <code>controlBus.electricMotorBus</code>. Electrical power is
absorbed through the motor-side DC pins instead of a public raw signal input.
</p>
</html>"));
end Motor;