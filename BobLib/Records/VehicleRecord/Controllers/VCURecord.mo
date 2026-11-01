within BobLib.Records.VehicleRecord.Controllers;

record VCURecord

  import Modelica.Units.SI;

  parameter SI.Torque tau_max = 220
    "VCU motoring torque limit";
  parameter SI.Power P_max_mot = 80000
    "VCU motoring power limit (FSAE tractive-system power cap)";
  parameter SI.Torque regenTorqueLimit = 220
    "Default generated-vehicle regen torque limit magnitude";
  parameter SI.Torque mechanicalBrakeTorqueLimit = 1500
    "Default generated-vehicle combined mechanical brake torque limit";
  parameter Real regenBrakeBlend(unit = "1", min = 0, max = 1) = 0
    "Default fraction of negative speed-control torque assigned to regen";
  parameter SI.AngularVelocity w_eps = 1.0
    "Direction-latch threshold and speed floor; match pMotor.w_eps";
  parameter Real motorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed in the VCU";

  parameter Real finalDriveRatio = 1
    "Final-drive ratio used to convert rear-axle torque command to motor torque" annotation(
    Dialog(tab = "Controllers", group = "Electric Drive Mapping"));
  annotation(
    Documentation(info = "<html>
<p>
Record <code>VCURecord</code> contains controller parameters and the standard
simulation regen and mechanical-brake command limits passed around the VCU
subsystem.
</p>
</html>"));
end VCURecord;
