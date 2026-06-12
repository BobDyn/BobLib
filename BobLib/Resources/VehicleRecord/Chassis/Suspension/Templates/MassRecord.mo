within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates;

record MassRecord
  import SI = Modelica.Units.SI;

  parameter SI.Mass m "Body mass";
  parameter SI.Position rCM[3] "Vector to center of mass, resolved in chassis frame";
  parameter SI.Inertia inertia[3,3] "Inertia tensor, resolved about frame at rCM";

end MassRecord;
