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

  Modelica.Blocks.Interfaces.RealInput P_elec
    "Electrical power into motor [W], positive while motoring" annotation(
      Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

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
  output SI.Power P_mech "Actual mechanical shaft power";
  output SI.Torque tau_cmd "Commanded motor torque";
  output SI.Torque tau_lim "Active motor torque limit";

equation
  w = motor.w;
  P_mech = motor.P_mech;
  tau_cmd = motor.tau_cmd;
  tau_lim = motor.tau_lim;

  connect(P_elec, motor.P_elec) annotation(
    Line(points = {{-120, 0}, {-56, 0}}, color = {0, 0, 127}));
  connect(motor.shaft, rotor.flange_a) annotation(
    Line(points = {{10, 0}, {35, 0}}));
  connect(rotor.flange_b, shaft_b.flange) annotation(
    Line(points = {{55, 0}, {100, 0}}));

  annotation(Documentation(info = "<html>
<p>
Thin adapter around <code>BobLibVehicleInterfaces.ElectricDrives.Internal.PowerLimitedMotor</code>.
The mechanical side follows <code>VehicleInterfaces.ElectricDrives.Interfaces.Base</code>
and includes the motor rotor inertia so the downstream driveline can remain a
pure final-drive/differential/halfshaft assembly.
</p>
</html>"));
end Motor;
