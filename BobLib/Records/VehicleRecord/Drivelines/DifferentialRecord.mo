within BobLib.Records.VehicleRecord.Drivelines;
record DifferentialRecord

  import Modelica.Units.SI;

  parameter Modelica.Units.SI.Torque preloadTorque(min = 0) = 20
    "Clutch transfer torque at zero input torque; output torque difference is twice this value";
  parameter Real driveLockingFraction(min = 0, max = 1) = 0.35
    "Drive-side output torque difference / input torque, excluding preload";
  parameter Real coastLockingFraction(min = 0, max = 1) = 0.15
    "Coast-side output torque difference / input torque, excluding preload";
  parameter Real driveTorqueBiasRatio(min = 1) = 3
    "Effective drive-side high/low output torque limit; requires calibration";
  parameter Real coastTorqueBiasRatio(min = 1) = 1
    "Effective coast-side high/low output torque limit; requires calibration";
  parameter Real slidingFrictionRatio(min = 1e-6, max = 1) = 1.0
    "Sliding/static clutch torque capacity";
  parameter Real driveSideTorqueSign = 1
    "Input torque sign selecting drive calibration (reverse operation uses the same sign convention)";
  annotation(Documentation(info = "<html><p>Component calibration.</p></html>"));
end DifferentialRecord;
