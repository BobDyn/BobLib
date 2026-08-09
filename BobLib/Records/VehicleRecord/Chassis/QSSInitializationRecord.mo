within BobLib.Records.VehicleRecord.Chassis;

record QSSInitializationRecord

  import SI = Modelica.Units.SI;

  parameter SI.Velocity referenceVelocity
    "Vehicle speed represented by this quasi-steady-state solution";
  parameter SI.Position chassisReferencePosition[3]
    "Solved world position of the sprung-mass reference frame";
  parameter SI.Angle chassisReferenceAngles[3]
    "Solved chassis orientation using the FreeMotion rotation sequence";
  parameter SI.Angle frontLeftLowerArmAngle;
  parameter SI.Angle frontRightLowerArmAngle;
  parameter SI.Angle rearLeftLowerArmAngle;
  parameter SI.Angle rearRightLowerArmAngle;
  parameter SI.Length frontSpringLength
    "Four-post spring length corresponding to the front lower-arm angles";
  parameter SI.Length rearSpringLength
    "Four-post spring length corresponding to the rear lower-arm angles";
  parameter SI.Force frontNormalLoad
    "Solved normal load per front corner";
  parameter SI.Force rearNormalLoad
    "Solved normal load per rear corner";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>QSSInitializationRecord</code> stores a coupled full-vehicle start
pose derived from four-post suspension solutions and the compliant ground
model. Lower-arm angles are the independent suspension state coordinates;
spring lengths and normal loads document the physical solution they represent.
</p>
</html>"));
end QSSInitializationRecord;
