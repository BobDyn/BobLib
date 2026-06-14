within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates;

record MassRecord
  import SI = Modelica.Units.SI;

  parameter SI.Mass m "Body mass";
  parameter SI.Position rCM[3] "Vector to center of mass, resolved in chassis frame";
  parameter SI.Inertia inertia[3,3] "Inertia tensor, resolved about frame at rCM";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>MassRecord</code> stores a body mass, center of mass, and inertia tensor.
</p>
<p>
It is the common mass-property container used by axle, sprung-mass, driver, and combined vehicle records.
</p>
</html>"));
end MassRecord;
