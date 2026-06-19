within BobLibVehicleInterfaces.Records.VehicleRecord.Controllers;

record VCURecord
  import SI = Modelica.Units.SI;

  parameter SI.Torque tau_max = 220
    "VCU motoring torque limit";
  parameter SI.Torque regenTorqueLimit = 220
    "Default generated-vehicle regen torque limit magnitude";
  parameter SI.Torque mechanicalBrakeTorqueLimit = 1500
    "Default generated-vehicle combined mechanical brake torque limit";
  parameter Real regenBrakeBlend(unit = "1", min = 0, max = 1) = 0
    "Default fraction of negative speed-control torque assigned to regen";
  parameter SI.AngularVelocity w_eps = 1.0
    "Low-speed regularization for VCU power conversion";
  parameter Real motorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed in the VCU";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>VCURecord</code> contains controller parameters and the standard
simulation regen and mechanical-brake command limits passed around the VCU
subsystem.
</p>
</html>"));
end VCURecord;
