within BobLib.Records.VehicleRecord.Brakes;
record BrakeRecord

  import Modelica.Units.SI;

  parameter Real differentialBrakeRatio(min = Modelica.Constants.small) = 1 "Brake shaft speed / differential carrier speed";

  parameter SI.Length pedalFullTravel = 0.05 "Travel at maximum brake torque";
  parameter SI.TranslationalSpringConstant pedalStiffness = 20000 "Effective hydraulic/pedal compliance";
  parameter SI.TranslationalDampingConstant pedalDamping = 100 "Pedal damping";
  parameter SI.Torque maxTorque = 1500
    "Maximum combined mechanical brake torque magnitude";
  parameter Real frontBrakeBias(unit = "1", min = 0, max = 1) = 0.55
    "Mechanical bias-bar fraction assigned to the front hydraulic circuit";
  parameter SI.AngularVelocity wRegularization = 0.1
    "Wheel-speed magnitude where braking torque reaches full sign";
  annotation(Documentation(info = "<html><p>Component calibration.</p></html>"));
end BrakeRecord;
