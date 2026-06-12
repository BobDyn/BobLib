within BobLib.Tests.Regression;

model MF52PureSlipSmoke
  import SI = Modelica.Units.SI;
  import Tire = BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52;
  import Vehicle = BobLib.Resources.VehicleDefn;

  parameter Vehicle.DWBCStabar_DWBCStabarRecord car;

  parameter SI.Force Fz = 654;
  parameter SI.Angle alpha = 0.08;
  parameter Real kappa = 0;
  parameter SI.Angle gamma = 0;
  parameter SI.Velocity Vx = 10;

  discrete SI.Force Fx;
  discrete SI.Force Fy;
  discrete SI.Torque Mx;
  discrete SI.Torque My;
  discrete SI.Torque Mz;
  discrete SI.Length pneumaticTrail;
  discrete SI.Length residualScrub;
  discrete SI.Torque MzReconstructed;

algorithm
  when initial() then
    (Fx, Fy, Mx, My, Mz, pneumaticTrail, residualScrub) :=
      Tire.Eval(Fz, alpha, kappa, gamma, Vx, car.pFrTireModel);
    MzReconstructed := -pneumaticTrail*Fy + residualScrub*Fx;
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end MF52PureSlipSmoke;
