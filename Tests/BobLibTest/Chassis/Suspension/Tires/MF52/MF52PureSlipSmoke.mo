within BobLibTest.Chassis.Suspension.Tires.MF52;

model MF52PureSlipSmoke

  import Modelica.Units.SI;
  import BobLib.Chassis.Suspension.Tires.MF52;
  import BobLib.Records.VehicleDefn;

  parameter VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord car "Vehicle calibration supplying the tire parameters for this test" annotation(Dialog(group = "Calibration"));

  parameter SI.Force Fz = 654 "Compressive tire normal load for the force-law evaluation" annotation(Dialog(group = "Operating settings"));
  parameter SI.Angle alpha = 0.08 "Tire slip angle used by the force-law evaluation, in radians" annotation(Dialog(group = "Operating settings"));
  parameter Real kappa = 0 "Longitudinal tire slip ratio" annotation(Dialog(group = "Operating settings"));
  parameter SI.Angle gamma = 0 "Tire camber angle used by the force-law evaluation, in radians" annotation(Dialog(group = "Operating settings"));
  parameter SI.Velocity Vx = 10 "Longitudinal tire speed used by the force-law evaluation" annotation(Dialog(group = "Operating settings"));

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
      MF52.Eval(Fz, alpha, kappa, gamma, Vx, car.chassis.frontAxle.tire.force);
    MzReconstructed := -pneumaticTrail*Fy + residualScrub*Fx;
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end MF52PureSlipSmoke;
