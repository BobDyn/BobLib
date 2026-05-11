within BobLib.Resources.StandardRecord;

record TransientEvalRecord
  import Modelica.SIunits;
  
  // Inputs
  Modelica.SIunits.Angle handwheelAngle "Steering wheel angle";

  // Kinematics
  Modelica.SIunits.Velocity velX "Longitudinal velocity";
  Modelica.SIunits.Velocity velY "Lateral velocity";
  Modelica.SIunits.AngularVelocity yawVel "Yaw velocity";
  Modelica.SIunits.Angle sideslip "Sideslip angle";
  
  // Accelerations
  Modelica.SIunits.Acceleration accX "Longitudinal acceleration";
  Modelica.SIunits.Acceleration accY "Lateral acceleration";
  
  // Vehicle response
  Modelica.SIunits.Angle roll "Vehicle roll angle";
  Modelica.SIunits.Torque handwheelTorque "Steering wheel torque";
  
end TransientEvalRecord;
