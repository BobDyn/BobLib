within BobLibVehicleInterfaces.Experiments.Standards.Templates;

model FourPostRrAxleDW_Direct
  import SI = Modelica.Units.SI;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

  extends BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_Direct;

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
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostRrAxleDW_Direct</code> specializes a rear double-wishbone axle with direct spring/damper actuation for the four-post rig.
</p>
<p>
It redeclares tire and wheel choices appropriate for fixture-driven suspension evaluation rather than full vehicle driving.
</p>
</html>"));
end FourPostRrAxleDW_Direct;
