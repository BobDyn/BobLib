within BobLib.ElectricDrives.Internal;

model PowerLimitedMotor

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  // Power command
  Modelica.Blocks.Interfaces.RealInput P_elec "Electrical power into motor [W], positive motoring and negative regen (connect from inverter P_out)" annotation(
    Placement(
      transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  // Torque output
  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  // Datasheet specs
  parameter SI.Voltage Vdc_max = 630 "Max battery voltage (EMRAX 228 MV) [Vdc]";
  parameter Real rpm_fullLoad_ref = 5300 "Full-load RPM at Vdc_max (datasheet)";
  parameter Real rpm_noLoad_ref = 6500 "No-load RPM at Vdc_max (datasheet)";
  parameter Real rpm_max_cont = 5500 "Max continuous rotation speed (datasheet) [rpm]";
  parameter Real rpm_max_peak = 6500 "Max peak speed for a few seconds (datasheet) [rpm]";
  parameter SI.Torque T_peak = 220 "Peak torque for a few seconds [Nm]";
  parameter SI.Torque T_cont = 130 "Continuous torque [Nm]";
  parameter SI.Current I_peak_2min = 360 "Max motor current for ~2 min if cooled [Arms]";
  parameter SI.Current I_cont = 180 "Continuous motor current [Arms]";
  parameter Real Kt_Nm_per_A = 0.61 "Torque per phase current (datasheet) [Nm/A]";
  parameter SI.Power P_mech_peak = 124e3 "Peak motor mechanical power capability [W]";
  parameter SI.Power P_cont_low = 75e3 "Low end continuous power band [W]";
  parameter SI.Power P_cont_high = 75e3 "High end continuous power band [W]";
  parameter Real eta_mot = 0.96 "Constant motoring efficiency approximation";
  parameter Real eta_reg = 0.95 "Constant regen efficiency approximation";
  parameter Real lossTable[:, 2] = [
    0,    0;
    1000, 200;
    2000, 550;
    3000, 1050;
    4000, 1750;
    5000, 2800
  ] "Free-run loss vs speed";
  parameter SI.Time peakTime = 120 "How long peak limits are allowed (seconds)";
  parameter Boolean enablePeakTimer = true "If true, peak limits ramp down after peakTime";

  // Numerical params
  parameter SI.AngularVelocity w_eps(min = Modelica.Constants.small) = 1e-3
    "Speed regularization and direction-latch threshold";
  parameter Boolean reverseLaunch = false
    "Select reverse torque direction when starting exactly from standstill";

  // Diagnostics
  SI.AngularVelocity w "Shaft speed [rad/s]";
  Real rpm "Shaft speed [rpm]";
  SI.Power P_loss_free "Free-run losses [W]";
  SI.Power P_mech_cmd "Commanded mechanical power after eff/loss [W]";
  SI.Power P_mech "Actual mechanical power at shaft [W]";
  SI.Torque tau_cmd "Commanded torque [Nm]";
  SI.Torque tau_lim "Active torque limit [Nm]";
  SI.Torque tau_lim_from_power "Torque limit from peak power [Nm]";
  SI.Torque tau_lim_from_current "Torque limit from current [Nm]";
  SI.Power P_cont_env "Continuous power envelope [W]";
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

protected
  function interp1

    input Real tbl[:, 2];
    input Real xq;
    output Real yq;

  algorithm
    yq := Modelica.Math.Vectors.interpolate(tbl[:, 1], tbl[:, 2], xq);
  end interp1;

  Real peakFactor "1 -> allow peak, 0 -> only continuous";
  SI.Torque T_allow;
  SI.Current I_allow;

  Real P_allow;
  Real P_mech_limited;
  SI.Torque tau_act(start = 0, fixed = true)
    "Actuated shaft torque";
  discrete Real shaftDirection(start = 1, fixed = true)
    "Latched shaft direction (+1 forward, -1 reverse)";
  SI.AngularVelocity w_eff
    "Direction-preserving regularized omega for power-to-torque conversion";

  parameter SI.Time tau_tau = 0.002 "Torque actuator time constant";

equation

  // Shaft speed
  w = der(shaft.phi);
  rpm = abs(w) * 60 / (2*pi);

  // Free-run losses (always dissipative)
  P_loss_free = interp1(lossTable, rpm);

  // Peak allowance factor (simple time-based derate)
  peakFactor =
    if not enablePeakTimer then 1
      else if time <= peakTime then 1
      else 0;

  // Allowed torque and current (peak -> continuous blend)
  T_allow = peakFactor*T_peak + (1 - peakFactor)*T_cont;
  I_allow = peakFactor*I_peak_2min + (1 - peakFactor)*I_cont;

  // Continuous power envelope vs speed
  P_cont_env =

    if noEvent(rpm <= 3000) then
      P_cont_low*(0.2 + 0.8*rpm/3000) // mild floor at low rpm
    elseif noEvent(rpm <= 5000) then
      P_cont_low + (P_cont_high - P_cont_low)*(rpm - 3000)/2000
    else
      P_cont_high;

  // Electrical -> mechanical power command (incl. losses)
  // Motoring: electrical supplies mech + losses
  // Regen: mechanical produces electrical, losses reduce recovery
  P_mech_cmd =

    if noEvent(P_elec >= 0) then
      noEvent(max(0, P_elec*eta_mot - P_loss_free))
    else
      noEvent(min(0, P_elec/eta_reg - P_loss_free));

  // Allowed mechanical power (peak vs continuous)
  P_allow =
    peakFactor*P_mech_peak
    + (1 - peakFactor)*P_cont_env;

  // Enforce power envelope (symmetric for motoring/regen)
  P_mech_limited =
    noEvent(max(min(P_mech_cmd,  P_allow), -P_allow));

  // Torque limits
  tau_lim_from_power = P_allow / max(abs(w), w_eps);
  tau_lim_from_current = Kt_Nm_per_A * I_allow;

  // Combined torque limit
  tau_lim =
    noEvent(min(T_allow,
        min(tau_lim_from_power,
            tau_lim_from_current)));

  // A power request alone has no direction information at standstill. Use the
  // configured launch direction there, then latch either established shaft
  // direction outside +/-w_eps. Holding the last direction inside the deadband
  // avoids torque-sign chatter as the shaft crosses zero.
  when initial() then
    shaftDirection =

      if abs(w) > w_eps then
        if w > 0 then 1 else -1
      else
        if reverseLaunch then -1 else 1;
  elsewhen w > w_eps then
    shaftDirection = 1;
  elsewhen w < -w_eps then
    shaftDirection = -1;
  end when;

  // Convert limited mechanical power to torque
  // The magnitude is smooth through zero while shaftDirection provides the
  // physically required sign. At established speed tau_cmd*w therefore has
  // the same sign as P_mech_limited for both directions of rotation.
  w_eff = shaftDirection*sqrt(w*w + w_eps*w_eps);
  tau_cmd = noEvent(max(min(P_mech_limited / w_eff, tau_lim), -tau_lim));

  // Torque actuator dynamics
  der(tau_act) = (tau_cmd - tau_act)/tau_tau;

  // Apply torque to shaft (Modelica sign convention)
  torque.tau = tau_act;

  // Actual mechanical power at shaft
  P_mech = tau_act * w;

  connect(torque.flange, shaft) annotation(
    Line(points = {{10, 0}, {100, 0}}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>PowerLimitedMotor</code> converts a power request and motor-speed state into limited shaft torque.
</p>
<p>
Positive power denotes motoring and negative power denotes regeneration,
independent of shaft rotation direction. The model latches the established
shaft direction after speed exceeds <code>w_eps</code> and holds it while speed
is inside that deadband to prevent torque-sign chatter. Since power alone does
not encode a direction at exactly zero speed, standstill uses forward launch by
default; set <code>reverseLaunch=true</code> for a reverse launch.
</p>
<p>
It represents the BobLib motor-side behavior behind the public electric-drive adapter while keeping the VehicleInterfaces contract at the package boundary.
</p>
</html>"));
end PowerLimitedMotor;
