within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

function MxCombinedEval
  import SI = Modelica.Units.SI;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MxPureRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.MxCombinedRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.SetupRecord;

  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.PureSlip.MxPureEval;

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

end MxCombinedEval;
