within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel;

model TransientSlip
  extends BaseSlipModel;

  import SI = Modelica.Units.SI;

  parameter SI.Force FNOMIN = 650
    "Nominal vertical load";

  parameter SI.Length UNLOADED_RADIUS = 0.2
    "Unloaded tire radius used for PAC2002 relaxation";

  parameter Real LFZO = 1
    "Scale factor of nominal load lambda_Fz0 [-]";

  parameter Real PTX1 = 0
    "Longitudinal relaxation coefficient p_Tx1 [-]";

  parameter Real PTX2 = 0
    "Longitudinal relaxation load dependency p_Tx2 [-]";

  parameter Real PTX3 = 0
    "Longitudinal relaxation exponential load dependency p_Tx3 [-]";

  parameter Real PTY1 = 0
    "Peak value of lateral relaxation length p_Ty1 [-]";

  parameter Real PTY2 = 0
    "Load at which lateral relaxation is extreme p_Ty2 [-]";

  parameter Real PKY3 = 0
    "Lateral relaxation camber sensitivity p_Ky3 [-]";

  parameter Real LSGKP = 1
    "Scale factor of longitudinal relaxation length [-]";

  parameter Real LSGAL = 1
    "Scale factor of lateral relaxation length [-]";

  parameter SI.Length sigma_kappa_default = 0.5
    "Fallback longitudinal relaxation length";

  parameter SI.Length sigma_alpha_default = 0.5
    "Fallback lateral relaxation length";

  parameter Real V_min = 0.5
    "Low-speed regularization velocity";

  parameter Real kappa_max = 2.0
    "Clamp for longitudinal slip";

  parameter Real alpha_max = 1.2
    "Clamp for slip angle [rad]";

  SI.Length u(nominal=0.1)
    "Longitudinal deformation state";

  SI.Length v(nominal=0.1)
    "Lateral deformation state";

  SI.Length sigma_kappa
    "PAC2002 longitudinal relaxation length";

  SI.Length sigma_alpha
    "PAC2002 lateral relaxation length";

protected
  SI.Velocity Vsx;
  SI.Velocity Vsy;
  SI.Velocity Vx_abs_eff;
  SI.Force Fz_eff;
  SI.Force Fz0;
  SI.Force Fz0_scaled;

  Real kappa_raw;
  Real alpha_raw;
  Real dfz;
  Real camber_factor;
  Real Vx_sign;

initial equation
  u = -sigma_kappa * (Vx - R0*omega) / noEvent(max(abs(Vx), V_min));
  v = sigma_alpha * (-Vy) / noEvent(max(abs(Vx), V_min));

equation
  Fz_eff = noEvent(max(Fz, 1e-3));
  Fz0 = noEvent(max(FNOMIN, 1e-3));
  Fz0_scaled = noEvent(max(FNOMIN*LFZO, 1e-3));
  dfz = (Fz_eff - Fz0_scaled) / Fz0_scaled;
  camber_factor = 1 - PKY3*abs(gamma);

  sigma_kappa =
    if PTX1 > 0 then
      noEvent(max(1e-4, Fz_eff * (PTX1 + PTX2*dfz) * exp(-PTX3*dfz) * (UNLOADED_RADIUS/Fz0) * LSGKP))
    else
      sigma_kappa_default;

  sigma_alpha =
    if PTY1 > 0 and PTY2 > 0 then
      noEvent(max(1e-4, PTY1 * sin(2*atan(Fz_eff/(PTY2*Fz0*LFZO))) * camber_factor * UNLOADED_RADIUS * LFZO * LSGAL))
    else
      sigma_alpha_default;

  // Slip velocities
  Vsx = Vx - R0 * omega;
  Vsy = -Vy;

  // Smooth regularized speed
  Vx_abs_eff = sqrt(Vx^2 + V_min^2);
  Vx_sign = noEvent(if Vx >= 0 then 1 else -1);

  // Transient dynamics (MF5.2 consistent)
  der(u) = -Vsx - (Vx_abs_eff / sigma_kappa) * u;
  der(v) = Vsy - (Vx_abs_eff / sigma_alpha) * v;

  // Output slips
  kappa_raw = (u / sigma_kappa) * Vx_sign;
  alpha_raw = atan2(v, sigma_alpha);

  kappa = noEvent(max(min(kappa_raw, kappa_max), -kappa_max));
  alpha = noEvent(max(min(alpha_raw, alpha_max), -alpha_max));

end TransientSlip;
