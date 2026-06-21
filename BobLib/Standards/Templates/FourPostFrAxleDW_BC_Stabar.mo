within BobLib.Standards.Templates;

model FourPostFrAxleDW_BC_Stabar

  import SI = Modelica.Units.SI;

  extends BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar;

  SI.Length leftSpringLength;
  SI.Length rightSpringLength;
  SI.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = stabar.spring.phi_rel;
end FourPostFrAxleDW_BC_Stabar;
