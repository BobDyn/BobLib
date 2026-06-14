within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates;

record AxleMassRecord
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  parameter MassRecord unsprungMass "Left unsprung mass record" annotation(
    Evaluate = false,
    Dialog(tab = "Mass Properties"));
  parameter MassRecord ucaMass "Left upper control arm mass record" annotation(
    Evaluate = false,
    Dialog(tab = "Mass Properties"));
  parameter MassRecord lcaMass "Left lower control arm mass record" annotation(
    Evaluate = false,
    Dialog(tab = "Mass Properties"));
  parameter MassRecord tieMass "Left tie rod mass record" annotation(
    Evaluate = false,
    Dialog(tab = "Mass Properties"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleMassRecord</code> groups unsprung and linkage mass properties for one axle side.
</p>
<p>
The detailed suspension models use these mass records to place inertias for upright, control arms, tie rod, wheel, and supporting linkages.
</p>
</html>"));
end AxleMassRecord;
