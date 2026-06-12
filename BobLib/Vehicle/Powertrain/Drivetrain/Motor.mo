within BobLib.Vehicle.Powertrain.Drivetrain;

model Motor
  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  // Power command
  Modelica.Blocks.Interfaces.RealInput P_elec "Electrical power into motor [W] (+motoring, −regen)  (connect from inverter P_out)" annotation(
    Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  // Torque output
  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft annotation(
    Placement(transformation(origin={100, 0}, extent={{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

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
  parameter Real eta_mot = 0.96 "Motoring efficiency placeholder (swap to 2D map later)";
  parameter Real eta_reg = 0.95 "Regen efficiency placeholder";
  parameter Real lossTable[:,2] = [
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
  parameter SI.AngularVelocity w_eps = 1e-3 "Small omega";

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
    input Real tbl[:,2];
    input Real xq;
    output Real yq;
  algorithm
    yq := Modelica.Math.Vectors.interpolate(tbl[:,1], tbl[:,2], xq);
  end interp1;

  Real peakFactor "1 -> allow peak, 0 -> only continuous";
  SI.Torque T_allow;
  SI.Current I_allow;

  Real P_allow;
  Real P_mech_limited;
  Real tau_act;
  SI.AngularVelocity w_eff "Effective omega for smooth power->torque conversion";

  parameter SI.Time tau_tau = 0.002 "Torque actuator time constant";


equation
  // Shaft speed
  w   = der(shaft.phi);
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
      P_cont_low*(0.2 + 0.8*rpm/3000)   // mild floor at low rpm
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
  tau_lim_from_power   = P_allow / max(abs(w), w_eps);
  tau_lim_from_current = Kt_Nm_per_A * I_allow;

  // Combined torque limit
  tau_lim =
    noEvent(min(T_allow,
        min(tau_lim_from_power,
            tau_lim_from_current)));

  // Convert limited mechanical power to torque
  w_eff = sqrt(w*w + w_eps*w_eps);
  tau_cmd = noEvent(max(min(P_mech_limited / w_eff, tau_lim), -tau_lim));

  // Torque actuator dynamics
  der(tau_act) = (tau_cmd - tau_act)/tau_tau;

  // Apply torque to shaft (Modelica sign convention)
  torque.tau = tau_act;

  // Actual mechanical power at shaft
  P_mech = tau_act * w;

  connect(torque.flange, shaft) annotation(
    Line(points = {{10, 0}, {100, 0}}));

end Motor;
