within BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.CombinedSlip;

function MyCombinedEval
  import SI = Modelica.Units.SI;

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MyPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.FxPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.MyCombinedRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  import BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.PureSlip.MyPureEval;

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

  annotation(
    Documentation(info = "<html>
<p>
Function <code>MyCombinedEval</code> evaluates the MF5.2 combined-slip contribution for rolling-resistance moment.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end MyCombinedEval;
