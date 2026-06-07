within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.PureSlip;

function MyPureEval
  import Modelica.SIunits;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MyPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FxPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  // Inputs
  input SIunits.Force Fz "Normal force";
  input SIunits.Force Fx "Longitudinal force";
  input SIunits.Velocity Vx "Longitudinal velocity";

  input MyPureRecord p;
  input FxPureRecord pFx;
  input SetupRecord setup;

  // Output
  output SIunits.Torque My;

protected
  Real eps = 1e-8;
  Real Vx_n;
  Real dfz;
  Real K_x;
  Real S_Hx;
  Real S_Vx;

algorithm
  if Fz > 1e-3 then

    if abs(p.QSY1) <= eps and abs(p.QSY2) <= eps then
      dfz := (Fz - setup.FNOMIN * pFx.LFZO) / (setup.FNOMIN * pFx.LFZO);

      K_x := Fz * (pFx.PKX1 + pFx.PKX2 * dfz)
             * exp(pFx.PKX3 * dfz)
             * pFx.LKX;

      S_Hx := (pFx.PHX1 + pFx.PHX2 * dfz) * pFx.LHX;

      S_Vx := Fz * (pFx.PVX1 + pFx.PVX2 * dfz)
              * pFx.LVX * pFx.LMUX;

      My := setup.UNLOADED_RADIUS * (S_Vx + K_x * S_Hx);
    else
      // Normalized velocity
      Vx_n := Vx / max(abs(p.Vref), eps);

      My :=
        setup.UNLOADED_RADIUS * Fz *
        (
          p.QSY1
          + p.QSY2 * (Fx / setup.FNOMIN)
          + p.QSY3 * abs(Vx_n)
          + p.QSY4 * Vx_n^4
        )
        * p.LMY;
    end if;

  else
    My := 0;
  end if;

end MyPureEval;
