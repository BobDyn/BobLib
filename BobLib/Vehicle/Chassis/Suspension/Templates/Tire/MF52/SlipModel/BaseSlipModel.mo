within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.SlipModel;

partial model BaseSlipModel
  import Modelica.SIunits;

  // Inputs (from BaseTire)
  input SIunits.Velocity Vx "Longitudinal velocity at contact patch";
  input SIunits.Velocity Vy "Lateral velocity at contact patch";
  input SIunits.AngularVelocity omega "Wheel angular speed";
  input SIunits.Length R0 "Unloaded tire radius";
  input SIunits.Force Fz "Tire normal load";
  input SIunits.Angle gamma "Inclination angle";

  // Outputs
  output SIunits.Angle alpha "Slip angle";
  output Real kappa "Slip ratio";

end BaseSlipModel;
