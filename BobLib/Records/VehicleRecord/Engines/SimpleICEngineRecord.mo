within BobLib.Records.VehicleRecord.Engines;
record SimpleICEngineRecord "Calibration of the sample throttle-to-torque engine"

  import Modelica.Units.SI;
  import Modelica.Constants.pi;

  parameter SI.Inertia flywheelInertia = 0.08
    "Sample crank/flywheel inertia";
  parameter SI.Torque peakTorque = 120
    "Peak positive engine torque at full accelerator request";
  parameter SI.Torque frictionTorque = 8
    "Simple speed-opposing friction torque magnitude";
  parameter SI.AngularVelocity redlineSpeed = 6500*2*pi/60
    "Speed where torque begins to roll off";
  parameter SI.AngularVelocity redlineSoftWidth = 300*2*pi/60
    "Speed range used for smooth redline torque cutoff";
  parameter SI.AngularVelocity w_eps = 1
    "Speed regularization for friction sign";
  parameter SI.Time torqueTimeConstant = 0.03
    "First-order torque response time";
end SimpleICEngineRecord;
