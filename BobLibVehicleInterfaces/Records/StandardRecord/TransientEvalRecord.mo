within BobLibVehicleInterfaces.Records.StandardRecord;

record TransientEvalRecord
  import SI = Modelica.Units.SI;

  // Inputs
  Modelica.Units.SI.Angle handwheelAngle "Steering wheel angle";

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

  annotation(
    Documentation(info = "<html>
<p>
Record <code>TransientEvalRecord</code> stores transient maneuver evaluation settings.
</p>
<p>
The record keeps standard experiment configuration data separate from executable model equations so templates can share the same setup.
</p>
</html>"));
end TransientEvalRecord;
