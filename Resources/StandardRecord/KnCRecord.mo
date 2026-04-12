within BobLib.Resources.StandardRecord;

record KnCRecord
  import Modelica.SIunits;

  SIunits.Length heave "Sprung mass heave";
  SIunits.Angle roll "Body roll angle [rad]";
  SIunits.Force fx "Total applied longitudinal force";
  SIunits.Force fy "Total applied lateral force";

  // Left wheel
  SIunits.Length leftSpringLength "Left spring length";
  SIunits.Angle leftGamma "Camber";
  SIunits.Angle leftToe "Toe";
  SIunits.Angle leftCaster "Caster";
  SIunits.Angle leftKpi "Kingpin inclination";
  SIunits.Length leftMechTrail "Mechanical trail";
  SIunits.Length leftMechScrub "Mechanical scrub radius";

  // Right wheel
  SIunits.Length rightSpringLength "Left spring length";
  SIunits.Angle rightGamma "Camber";
  SIunits.Angle rightToe "Toe";
  SIunits.Angle rightCaster "Caster";
  SIunits.Angle rightKpi "Kingpin inclination";
  SIunits.Length rightMechTrail "Mechanical trail";
  SIunits.Length rightMechScrub "Mechanical scrub radius";

  // Axle-level
  SIunits.Force jackingForce "Total axle jacking force";
  SIunits.Angle stabarAngle "Angular deflection of torsion bar";
  
end KnCRecord;
