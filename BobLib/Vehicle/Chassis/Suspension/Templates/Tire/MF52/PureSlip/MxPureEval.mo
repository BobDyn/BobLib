within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.PureSlip;

function MxPureEval
  import SI = Modelica.Units.SI;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MxPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

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

end MxPureEval;
