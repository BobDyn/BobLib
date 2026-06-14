within BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.PureSlip;

function FyPureEval
  import SI = Modelica.Units.SI;

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FyPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  input SI.Force Fz;
  input SI.Angle alpha;
  input SI.Angle gamma;

  input FyPureRecord p;
  input SetupRecord setup;

  output SI.Force Fy;

protected
  Real IA_y;
  Real dfz;
  Real mu_y;

  Real C;
  Real D;
  Real K;
  Real B;
  Real Sh;
  Real Sv;
  Real SA;
  Real E;

algorithm
  if Fz > 1e-3 then
    IA_y := gamma * p.LGAY;

    dfz := (Fz - setup.FNOMIN * p.LFZO) / (setup.FNOMIN * p.LFZO);

    mu_y := (p.PDY1 + p.PDY2 * dfz) * (1 - p.PDY3 * IA_y^2) * p.LMUY;

    C := p.PCY1 * p.LCY;
    D := mu_y * Fz;

    K := p.PKY1 * setup.FNOMIN *
         sin(2 * atan(Fz / (p.PKY2 * setup.FNOMIN * p.LFZO))) *
         (1 - p.PKY3 * abs(IA_y)) *
         p.LFZO * p.LKY;

    B := K / (C * D + 1e-8);

    Sh := (p.PHY1 + p.PHY2 * dfz) * p.LHY + p.PHY3 * IA_y;
    Sv := Fz * ((p.PVY1 + p.PVY2 * dfz) * p.LVY +
                (p.PVY3 + p.PVY4 * dfz) * IA_y) * p.LMUY;

    SA := alpha + Sh;

    E := (p.PEY1 + p.PEY2 * dfz) *
         (1 - (p.PEY3 + p.PEY4 * IA_y) * sign(SA)) *
         p.LEY;
    E := min(E, 1);

    Fy := D * sin(C * atan(B * SA - E * (B * SA - atan(B * SA)))) + Sv;

  else
    Fy := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>FyPureEval</code> evaluates the MF5.2 pure-slip contribution for lateral force.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end FyPureEval;
