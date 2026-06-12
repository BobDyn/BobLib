within BobLib.Standards.Templates;

model FourPostRrAxleDW_BC_Stabar
  import Modelica.SIunits;

  extends BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar;

  SIunits.Length leftSpringLength;
  SIunits.Length rightSpringLength;
  SIunits.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = stabar.spring.phi_rel;
end FourPostRrAxleDW_BC_Stabar;
