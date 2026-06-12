within BobLib.Standards.Templates;

model FourPostRrAxleDW_Direct
  import SI = Modelica.Units.SI;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

  extends BobLib.Vehicle.Chassis.Suspension.RrAxleDW_Direct;

  parameter StabarRecord pStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);
  SI.Length leftSpringLength;
  SI.Length rightSpringLength;
  SI.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = 0;
end FourPostRrAxleDW_Direct;
