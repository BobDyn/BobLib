within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

function MyCombinedEval

  import SI = Modelica.Units.SI;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MyPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FxPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.MyCombinedRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MyPureEval;

  input SI.Force Fz;
  input SI.Force Fx;
  input SI.Velocity Vx;

  input MyPureRecord pPure;
  input FxPureRecord pFx;
  input MyCombinedRecord pComb;
  input SetupRecord setup;

  output SI.Torque My;

algorithm
  if Fz > 1e-3 then
    My := MyPureEval(Fz, Fx, Vx, pPure, pFx, setup);
  else
    My := 0;
  end if;

end MyCombinedEval;
