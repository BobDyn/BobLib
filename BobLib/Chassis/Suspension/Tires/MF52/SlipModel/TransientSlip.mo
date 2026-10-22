within BobLib.Chassis.Suspension.Tires.MF52.SlipModel;

model TransientSlip

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.RelaxationRecord pSlip "Tire relaxation and low-speed slip calibration" annotation(Dialog(group = "Calibration"));

  extends BaseSlipModel;

  SI.Length u(nominal = 0.1)
    "Longitudinal deformation state";

  SI.Length v(nominal = 0.1)
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
  Modelica.Units.SI.Angle alpha_raw;
  Real dfz;
  Real camber_factor;

initial equation
  u = -sigma_kappa * (Vx - R0*omega) / Vx_abs_eff;
  v = sigma_alpha * (-Vy) / Vx_abs_eff;

equation
  assert(
    pSlip.V_min > 0 and pSlip.sigma_kappa_default > 0 and pSlip.sigma_alpha_default > 0 and pSlip.FNOMIN > 0 and pSlip.LFZO > 0 and pSlip.UNLOADED_RADIUS > 0,
    "TransientSlip: speed regularization, relaxation lengths, nominal load, load scale and radius must be positive");
  Fz_eff = noEvent(max(Fz, 1e-3));
  Fz0 = noEvent(max(pSlip.FNOMIN, 1e-3));
  Fz0_scaled = noEvent(max(pSlip.FNOMIN*pSlip.LFZO, 1e-3));
  dfz = (Fz_eff - Fz0_scaled) / Fz0_scaled;
  camber_factor = 1 - pSlip.PKY3*abs(gamma);

  sigma_kappa =

    if pSlip.PTX1 > 0 then
      noEvent(max(1e-4, Fz_eff * (pSlip.PTX1 + pSlip.PTX2*dfz) * exp(-pSlip.PTX3*dfz) * (pSlip.UNLOADED_RADIUS/Fz0) * pSlip.LSGKP))
    else
      pSlip.sigma_kappa_default;

  sigma_alpha =

    if pSlip.PTY1 > 0 and pSlip.PTY2 > 0 then
      noEvent(max(1e-4, pSlip.PTY1 * sin(2*atan(Fz_eff/(pSlip.PTY2*Fz0*pSlip.LFZO))) * camber_factor * pSlip.UNLOADED_RADIUS * pSlip.LFZO * pSlip.LSGAL))
    else
      pSlip.sigma_alpha_default;

  // Slip velocities
  Vsx = Vx - R0 * omega;
  Vsy = -Vy;

  // Smooth regularized speed
  Vx_abs_eff = sqrt(Vx^2 + pSlip.V_min^2);

  // Transient dynamics (MF5.2 consistent)
  der(u) = -Vsx - (Vx_abs_eff / sigma_kappa) * u;
  der(v) = Vsy - (Vx_abs_eff / sigma_alpha) * v;

  // Output slips
  kappa_raw = u / sigma_kappa;
  alpha_raw = atan2(v, sigma_alpha);

  kappa = noEvent(max(min(kappa_raw, pSlip.kappa_max), -pSlip.kappa_max));
  alpha = noEvent(max(min(alpha_raw, pSlip.alpha_max), -pSlip.alpha_max));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>TransientSlip</code> computes tire slip with relaxation dynamics.
</p>
<p>
It introduces transient longitudinal and lateral slip states so users can study tire relaxation behavior at the cost of additional dynamics.
</p>
</html>"));
end TransientSlip;
