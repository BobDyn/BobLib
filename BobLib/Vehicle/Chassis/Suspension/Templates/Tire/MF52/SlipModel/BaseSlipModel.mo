within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel;

partial model BaseSlipModel
  import SI = Modelica.Units.SI;

  // Inputs (from BaseTire)
  input SI.Velocity Vx "Longitudinal velocity at contact patch";
  input SI.Velocity Vy "Lateral velocity at contact patch";
  input SI.AngularVelocity omega "Wheel angular speed";
  input SI.Length R0 "Unloaded tire radius";
  input SI.Force Fz "Tire normal load";
  input SI.Angle gamma "Inclination angle";

  // Outputs
  output SI.Angle alpha "Slip angle";
  output Real kappa "Slip ratio";

end BaseSlipModel;
