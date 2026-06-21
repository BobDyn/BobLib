within BobLib.Chassis.Suspension.Tires.MF52.PureSlip;

function MxPureEval

  import SI = Modelica.Units.SI;

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MxPureRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  // Tire inputs
  input SI.Force Fz "Normal force acting on tire";
  input SI.Force Fy "Lateral force acting on tire";
  input SI.Angle gamma "Inclination angle (camber), radians";

  input MxPureRecord p;
  input SetupRecord setup;

  // Output
  output SI.Torque Mx;

algorithm
  if Fz > 1e-3 then

    Mx :=
      setup.UNLOADED_RADIUS * Fz *
      (
        p.QSX1 * p.LVMX
        +
        (-p.QSX2 * gamma + p.QSX3 * Fy / setup.FNOMIN) * p.LMX
      );

  else
    Mx := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>MxPureEval</code> evaluates the MF5.2 pure-slip contribution for overturning moment.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end MxPureEval;
