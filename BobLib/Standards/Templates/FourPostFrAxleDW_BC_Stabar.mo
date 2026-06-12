within BobLib.Standards.Templates;

model FourPostFrAxleDW_BC_Stabar
  import Modelica.SIunits;

  extends BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar;

  SIunits.Length leftSpringLength;
  SIunits.Length rightSpringLength;
  SIunits.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = stabar.spring.phi_rel;
end FourPostFrAxleDW_BC_Stabar;
