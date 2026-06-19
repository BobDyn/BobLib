within BobLibVehicleInterfaces.Records.StandardRecord;

record FourPostEvalRecord
  import SI = Modelica.Units.SI;

  SI.Length heave "Sprung mass heave";
  SI.Angle roll "Body roll angle [rad]";
  SI.Force fx "Total applied longitudinal force";
  SI.Force fy "Total applied lateral force";

  // Left wheel
  SI.Length leftSpringLength "Left spring length";
  SI.Force leftFz "Left contact-patch normal force";
  SI.Angle leftGamma "Camber";
  SI.Angle leftToe "Toe";
  SI.Angle leftCaster "Caster";
  SI.Angle leftKpi "Kingpin inclination";
  SI.Length leftMechTrail "Mechanical trail";
  SI.Length leftMechScrub "Mechanical scrub radius";

  // Right wheel
  SI.Length rightSpringLength "Right spring length";
  SI.Force rightFz "Right contact-patch normal force";
  SI.Angle rightGamma "Camber";
  SI.Angle rightToe "Toe";
  SI.Angle rightCaster "Caster";
  SI.Angle rightKpi "Kingpin inclination";
  SI.Length rightMechTrail "Mechanical trail";
  SI.Length rightMechScrub "Mechanical scrub radius";

  // Axle-level
  SI.Force jackingForce "Total axle jacking force";
  SI.Angle stabarAngle "Angular deflection of torsion bar";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>FourPostEvalRecord</code> stores four-post evaluation settings.
</p>
<p>
The record keeps standard experiment configuration data separate from executable model equations so templates can share the same setup.
</p>
</html>"));
end FourPostEvalRecord;
