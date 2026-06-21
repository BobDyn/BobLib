within BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.CombinedSlip;

function MzCombinedEval

  import SI = Modelica.Units.SI;

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MzPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.MzCombinedRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FyPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FxPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  import BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.PureSlip.MzPureEval;

  input SI.Force Fz;
  input SI.Force Fx;
  input SI.Force Fy;
  input SI.Angle alpha;
  input SI.DimensionlessRatio kappa;
  input SI.Angle gamma;

  input FyPureRecord pFy;
  input FxPureRecord pFx;
  input MzPureRecord pPure;
  input MzCombinedRecord pComb;
  input SetupRecord setup;

  output SI.Torque Mz;
  output SI.Length t;
  output SI.Length s;

protected
  Real dfz;
  Real mu_y;

  Real D_VySR;
  Real S_VySR;

  Real Fy_eff;
  Real Mz_pure;

algorithm
  if Fz > 1e-3 then

    // ------------------------------------------------------------
    // Load normalization
    // ------------------------------------------------------------
    dfz := (Fz - setup.FNOMIN * pFy.LFZO) / (setup.FNOMIN * pFy.LFZO);

    mu_y := (pFy.PDY1 + pFy.PDY2 * dfz)

            * (1 - pFy.PDY3 * (gamma * pFy.LGAY)^2)
            * pFy.LMUY;

    // ------------------------------------------------------------
    // Combined Fy shift
    // ------------------------------------------------------------
    D_VySR := mu_y * Fz

              * (pComb.RVY1 + pComb.RVY2 * dfz + pComb.RVY3 * gamma)
              * cos(atan(pComb.RVY4 * alpha));

    S_VySR := D_VySR

              * sin(pComb.RVY5 * atan(pComb.RVY6 * kappa))
              * pFy.LVYKA;

    // Effective Fy used in Mz model
    Fy_eff := Fy - S_VySR;

    // ------------------------------------------------------------
    // Pure aligning moment
    // ------------------------------------------------------------
    Mz_pure := MzPureEval(
      Fz,
      Fy_eff,
      alpha,
      kappa,
      gamma,
      pFy,
      pFx,
      pPure,
      setup
    );

    // ------------------------------------------------------------
    // Pneumatic trail extraction
    // ------------------------------------------------------------
    if abs(Fy_eff) > 1e-6 then
      t := -Mz_pure / Fy_eff;
    else
      t := 0;
    end if;

    // ------------------------------------------------------------
    // Residual arm (already correct in your model)
    // ------------------------------------------------------------
    s := (pComb.SSZ1
         + pComb.SSZ2 * (Fy / setup.FNOMIN)
         + (pComb.SSZ3 + pComb.SSZ4 * dfz) * gamma)

         * setup.UNLOADED_RADIUS * pComb.LS;

    // ------------------------------------------------------------
    // Final Mz
    // ------------------------------------------------------------
    Mz := Mz_pure + s * Fx;

  else
    Mz := 0;
    t := 0;
    s := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>MzCombinedEval</code> evaluates the MF5.2 combined-slip contribution for aligning moment.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end MzCombinedEval;
