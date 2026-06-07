within BobLib.Tests.Regression;

model MF52PureSlipSmoke
  import Modelica.SIunits;
  import Tire = BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52;
  import Vehicle = BobLib.Resources.VehicleDefn;

  parameter Vehicle.DWBCStabar_DWBCStabarRecord car;

  parameter SIunits.Force Fz = 654;
  parameter SIunits.Angle alpha = 0.08;
  parameter Real kappa = 0;
  parameter SIunits.Angle gamma = 0;
  parameter SIunits.Velocity Vx = 10;

  discrete SIunits.Force Fx;
  discrete SIunits.Force Fy;
  discrete SIunits.Torque Mx;
  discrete SIunits.Torque My;
  discrete SIunits.Torque Mz;
  discrete SIunits.Length pneumaticTrail;
  discrete SIunits.Length residualScrub;
  discrete SIunits.Torque MzReconstructed;

algorithm
  when initial() then
    (Fx, Fy, Mx, My, Mz, pneumaticTrail, residualScrub) :=
      Tire.Eval(Fz, alpha, kappa, gamma, Vx, car.pFrTireModel);
    MzReconstructed := -pneumaticTrail*Fy + residualScrub*Fx;
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end MF52PureSlipSmoke;
