within BobLib.Standards.Templates;

model FourPostRrAxleDW_Direct
  import Modelica.SIunits;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

  extends BobLib.Vehicle.Chassis.Suspension.RrAxleDW_Direct;

  parameter StabarRecord pStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);
  SIunits.Length leftSpringLength;
  SIunits.Length rightSpringLength;
  SIunits.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = 0;
end FourPostRrAxleDW_Direct;
