within BobLib.Chassis.Suspension.Tires.MF52.PureSlip;

function MzPureEval

  import SI = Modelica.Units.SI;

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MzPureRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FyPureRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FxPureRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  input SI.Force Fz;
  input SI.Force Fy;
  input SI.Angle alpha;
  input SI.DimensionlessRatio kappa;
  input SI.Angle gamma;

  input FyPureRecord pFy;
  input FxPureRecord pFx;
  input MzPureRecord p;
  input SetupRecord setup;

  output SI.Torque Mz_pure;

protected
  Real dfz;
  Real mu_y;
  Real C_y;
  Real K_y;
  Real B_y;
  Real K_x;
  Real IA_y;
  Real S_Hy;
  Real S_Vy;

  Real IA_z;
  Real eps = 1e-8;

  Real D_t;
  Real C_t;
  Real B_t;
  Real E_t;
  Real S_Ht;
  Real SA_t;
  Real SA_t_eq;

  Real SA_r;
  Real SA_r_eq;

  Real t;
  Real D_r;
  Real B_r;
  Real M_zr;

algorithm
  if Fz > 1e-3 then

    // ------------------------------------------------------------
    // Load normalization
    // ------------------------------------------------------------
    dfz := (Fz - setup.FNOMIN * pFy.LFZO) / (setup.FNOMIN * pFy.LFZO);

    // ------------------------------------------------------------
    // Fy internals
    // ------------------------------------------------------------
    IA_y := gamma * pFy.LGAY;

    C_y := pFy.PCY1 * pFy.LCY;

    mu_y := (pFy.PDY1 + pFy.PDY2 * dfz)

            * (1 - pFy.PDY3 * IA_y^2)
            * pFy.LMUY;

    K_y := pFy.PKY1 * setup.FNOMIN *
           sin(2 * atan(Fz / (pFy.PKY2 * setup.FNOMIN * pFy.LFZO))) *
           (1 - pFy.PKY3 * abs(IA_y)) *
           pFy.LFZO * pFy.LKY;

    B_y := K_y / (C_y * mu_y * Fz + eps);

    S_Hy := (pFy.PHY1 + pFy.PHY2 * dfz) * pFy.LHY
            + pFy.PHY3 * IA_y;

    S_Vy := Fz * ((pFy.PVY1 + pFy.PVY2 * dfz) * pFy.LVY
             + (pFy.PVY3 + pFy.PVY4 * dfz) * IA_y) * pFy.LMUY;

    // ------------------------------------------------------------
    // Fx stiffness
    // ------------------------------------------------------------
    K_x := Fz * (pFx.PKX1 + pFx.PKX2 * dfz)

           * exp(pFx.PKX3 * dfz)
           * pFx.LKX;

    // ------------------------------------------------------------
    // Trail model
    // ------------------------------------------------------------
    IA_z := gamma;

    D_t := Fz * (p.QDZ1 + p.QDZ2 * dfz)

           * (1 + p.QDZ3 * IA_z * p.LGAZ + p.QDZ4 * (IA_z * p.LGAZ)^2)
           * (setup.UNLOADED_RADIUS / setup.FNOMIN)
           * p.LTR;

    C_t := p.QCZ1;

    B_t := (p.QBZ1 + p.QBZ2 * dfz + p.QBZ3 * dfz^2)

           * (1 + p.QBZ4 * IA_z * p.LGAZ + p.QBZ5 * abs(IA_z * p.LGAZ))
           * p.LKY / p.LMUY;

    S_Ht := p.QHZ1 + p.QHZ2 * dfz
            + (p.QHZ3 + p.QHZ4 * dfz) * IA_z * p.LGAZ;

    SA_t := alpha + S_Ht;

    E_t := (p.QEZ1 + p.QEZ2 * dfz + p.QEZ3 * dfz^2)

           * (1 + (p.QEZ4 + p.QEZ5 * IA_z * p.LGAZ)
           * (2 / Modelica.Constants.pi) * atan(B_t * C_t * SA_t));
    E_t := min(E_t, 1);

    SA_t_eq :=
      atan(sqrt((tan(SA_t))^2 + (K_x / (K_y + eps))^2 * kappa^2))

      * sign(SA_t);

    t := D_t

         * cos(C_t * atan(B_t * SA_t_eq - E_t * (B_t * SA_t_eq - atan(B_t * SA_t_eq))))
         * cos(alpha);

    // ------------------------------------------------------------
    // Residual torque
    // ------------------------------------------------------------
    SA_r := alpha + S_Hy + S_Vy / (K_y + eps);

    SA_r_eq :=
      atan(sqrt((tan(SA_r))^2 + (K_x / (K_y + eps))^2 * kappa^2))

      * sign(SA_r);

    D_r := Fz * ((p.QDZ6 + p.QDZ7 * dfz) * p.LRES
           + (p.QDZ8 + p.QDZ9 * dfz) * IA_z * p.LGAZ)

           * setup.UNLOADED_RADIUS * p.LMUY;

    B_r := p.QBZ9 * p.LKY / p.LMUY + p.QBZ10 * B_y * C_y;

    M_zr := D_r * cos(atan(B_r * SA_r_eq)) * cos(alpha);

    // ------------------------------------------------------------
    // Final
    // ------------------------------------------------------------
    Mz_pure := -t * Fy + M_zr;

  else
    Mz_pure := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>MzPureEval</code> evaluates the MF5.2 pure-slip contribution for aligning moment.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end MzPureEval;
