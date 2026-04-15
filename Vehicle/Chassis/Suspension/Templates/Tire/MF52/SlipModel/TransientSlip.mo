within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel;

model TransientSlip
  extends BaseSlipModel;

  import Modelica.SIunits;

  parameter SIunits.Length sigma_kappa = 0.5
    "Longitudinal relaxation length";

  parameter SIunits.Length sigma_alpha = 0.5
    "Lateral relaxation length";

  parameter Real V_min = 0.5
    "Low-speed regularization velocity";

  parameter Real kappa_max = 2.0
    "Clamp for longitudinal slip";

  parameter Real alpha_max = 1.2
    "Clamp for slip angle [rad]";

  SIunits.Length u(nominal=0.1)
    "Longitudinal deformation state";

  SIunits.Length v(nominal=0.1)
    "Lateral deformation state";

protected
  SIunits.Velocity Vsx;
  SIunits.Velocity Vsy;
  SIunits.Velocity Vx_eff;

  Real kappa_raw;
  Real alpha_raw;

initial equation
  u = -sigma_kappa * (Vx - R0*omega) / max(abs(Vx), V_min);
  v = -sigma_alpha * Vy / max(abs(Vx), V_min);

equation
  // Slip velocities
  Vsx = Vx - R0 * omega;
  Vsy = Vy;

  // Smooth regularized speed
  Vx_eff = sqrt(Vx^2 + V_min^2) * (if Vx >= 0 then 1 else -1);

  // Transient dynamics (MF5.2 consistent)
  der(u) = -Vsx - (Vx_eff / sigma_kappa) * u;
  der(v) = -Vsy - (Vx_eff / sigma_alpha) * v;

  // Output slips
  kappa_raw = u / sigma_kappa;
  alpha_raw = atan2(v, sigma_alpha);

  kappa = max(min(kappa_raw, kappa_max), -kappa_max);
  alpha = max(min(alpha_raw, alpha_max), -alpha_max);

end TransientSlip;
