within BobLib.Resources.StandardRecord;

record SteadyStateEvalRecord

  import SI = Modelica.Units.SI;

  // Inputs
  Modelica.Units.SI.Angle handwheelAngle "Steering wheel angle";
  Modelica.Units.SI.Angle leftSteerAngle "Left road wheel angle";
  Modelica.Units.SI.Angle rightSteerAngle "Right road wheel angle";

  // Kinematics
  Modelica.Units.SI.Velocity velX "Longitudinal velocity";
  Modelica.Units.SI.Velocity velY "Lateral velocity";
  Modelica.Units.SI.AngularVelocity yawVel "Yaw velocity";
  Modelica.Units.SI.Angle sideslip "Sideslip angle";

  // Accelerations
  Modelica.Units.SI.Acceleration accX "Longitudinal acceleration";
  Modelica.Units.SI.Acceleration accY "Lateral acceleration";

  // Vehicle response
  Modelica.Units.SI.Angle roll "Vehicle roll angle";
  Modelica.Units.SI.Torque handwheelTorque "Steering wheel torque";

  // Derived
  Real curvature "Path curvature (1/m)";

end SteadyStateEvalRecord;
