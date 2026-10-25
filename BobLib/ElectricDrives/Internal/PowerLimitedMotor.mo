within BobLib.ElectricDrives.Internal;

model PowerLimitedMotor

  import Modelica.Units.SI;
  import Modelica.Constants.pi;

  parameter BobLib.Records.VehicleRecord.ElectricDrives.MotorRecord pMotor(w_eps = 1e-3) "Motor torque, power, efficiency and rotor calibration" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.ElectricMotor;

  // Power command
  Modelica.Blocks.Interfaces.RealInput P_elec "Electrical power into motor [W], positive motoring and negative regen (connect from inverter P_out)" annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  // Torque output
  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  parameter Boolean useSupport = false "Expose stator reaction" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  Modelica.Mechanics.Rotational.Interfaces.Support support(phi = supportAngle) if useSupport annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}})));

  // Datasheet specs

  // Numerical params

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

  Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport = useSupport) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

protected
  SI.Angle supportAngle;
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

initial equation
  assert(pMotor.w_eps > 0 and pMotor.tau_tau > 0,
    "PowerLimitedMotor: w_eps and tau_tau must be positive");
  assert(pMotor.eta_mot > 0 and pMotor.eta_mot <= 1 and pMotor.eta_reg > 0 and pMotor.eta_reg <= 1,
    "PowerLimitedMotor: efficiencies must be in (0, 1]");
  assert(min({pMotor.T_peak, pMotor.T_cont, pMotor.I_peak_2min, pMotor.I_cont,
    pMotor.Kt_Nm_per_A, pMotor.P_mech_peak, pMotor.P_cont_low, pMotor.P_cont_high, pMotor.peakTime}) >= 0,
    "PowerLimitedMotor: torque, current, power and duration limits must be non-negative");
  assert(size(pMotor.lossTable, 1) >= 2 and min(pMotor.lossTable[:, 2]) >= 0,
    "PowerLimitedMotor: lossTable needs at least two points and non-negative losses");
  for i in 2:size(pMotor.lossTable, 1) loop
    assert(pMotor.lossTable[i, 1] > pMotor.lossTable[i - 1, 1],
      "PowerLimitedMotor: lossTable speed breakpoints must be strictly increasing");
  end for;

equation
  connect(torque.support, support) annotation(
    Line(visible = useSupport, points = {{0, -10}, {0, -100}}, color = {0, 0, 0}));

  // Shaft speed
  if not useSupport then
    supportAngle = 0;
  end if;
  w = der(shaft.phi-supportAngle);
  rpm = abs(w) * 60 / (2*pi);

  // Free-run losses (always dissipative)
  P_loss_free = interp1(pMotor.lossTable, rpm);

  // Peak allowance factor (simple time-based derate)
  peakFactor =
    if not pMotor.enablePeakTimer then 1
      else if time <= pMotor.peakTime then 1
      else 0;

  // Allowed torque and current (peak -> continuous blend)
  T_allow = peakFactor*pMotor.T_peak + (1 - peakFactor)*pMotor.T_cont;
  I_allow = peakFactor*pMotor.I_peak_2min + (1 - peakFactor)*pMotor.I_cont;

  // Continuous power envelope vs speed
  P_cont_env =

    if noEvent(rpm <= 3000) then
      pMotor.P_cont_low*(0.2 + 0.8*rpm/3000) // mild floor at low rpm
    elseif noEvent(rpm <= 5000) then
      pMotor.P_cont_low + (pMotor.P_cont_high - pMotor.P_cont_low)*(rpm - 3000)/2000
    else
      pMotor.P_cont_high;

  // Electrical -> mechanical power command (incl. losses)
  // Motoring: electrical supplies mech + losses
  // Regen: mechanical produces electrical, losses reduce recovery
  P_mech_cmd =

    if noEvent(P_elec >= 0) then
      noEvent(max(0, P_elec*pMotor.eta_mot - P_loss_free))
    else
      noEvent(min(0, P_elec/pMotor.eta_reg - P_loss_free));

  // Allowed mechanical power (peak vs continuous)
  P_allow =
    peakFactor*pMotor.P_mech_peak
    + (1 - peakFactor)*P_cont_env;

  // Enforce power envelope (symmetric for motoring/regen)
  P_mech_limited =
    noEvent(max(min(P_mech_cmd,  P_allow), -P_allow));

  // Torque limits
  tau_lim_from_power = P_allow / max(abs(w), pMotor.w_eps);
  tau_lim_from_current = pMotor.Kt_Nm_per_A * I_allow;

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

      if abs(w) > pMotor.w_eps then
        if w > 0 then 1 else -1
      else
        if pMotor.reverseLaunch then -1 else 1;
  elsewhen w > pMotor.w_eps then
    shaftDirection = 1;
  elsewhen w < -pMotor.w_eps then
    shaftDirection = -1;
  end when;

  // Convert limited mechanical power to torque
  // The magnitude is smooth through zero while shaftDirection provides the
  // physically required sign. At established speed tau_cmd*w therefore has
  // the same sign as P_mech_limited for both directions of rotation.
  w_eff = shaftDirection*sqrt(w*w + pMotor.w_eps*pMotor.w_eps);
  tau_cmd = noEvent(max(min(P_mech_limited / w_eff, tau_lim), -tau_lim));

  // Torque actuator dynamics
  der(tau_act) = (tau_cmd - tau_act)/pMotor.tau_tau;

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
default; set <code>pMotor.reverseLaunch=true</code> for a reverse launch.
</p>
<p>
It represents the BobLib motor-side behavior behind the public electric-drive adapter while keeping the VehicleInterfaces contract at the package boundary.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)));
end PowerLimitedMotor;
