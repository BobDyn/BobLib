within BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.CombinedSlip;

function MxCombinedEval

  import SI = Modelica.Units.SI;

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MxPureRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.MxCombinedRecord;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  import BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.PureSlip.MxPureEval;

  input SI.Force Fz;
  input SI.Force Fy;
  input SI.Angle gamma;

  input MxPureRecord pPure;
  input MxCombinedRecord pComb;
  input SetupRecord setup;

  output SI.Torque Mx;

algorithm
  if Fz > 1e-3 then
    Mx := MxPureEval(Fz, Fy, gamma, pPure, setup);
  else
    Mx := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>MxCombinedEval</code> evaluates the MF5.2 combined-slip contribution for overturning moment.
</p>
<p>
The function is intentionally narrow so each tire force or moment equation can be maintained and tested independently.
</p>
</html>"));
end MxCombinedEval;
