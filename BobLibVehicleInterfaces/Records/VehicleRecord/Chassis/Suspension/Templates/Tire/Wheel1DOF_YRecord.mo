within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire;

record Wheel1DOF_YRecord
  import SI = Modelica.Units.SI;

  // Mass properties
  parameter SI.Inertia wheelJ "Effective inertia of rotating mass" annotation(Dialog(group = "Mass Properties"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>Wheel1DOF_YRecord</code> stores parameters for the corresponding wheel degree-of-freedom model.
</p>
<p>
It keeps wheel inertia, vertical stiffness, damping, and related values separate from the tire force equations.
</p>
</html>"));
end Wheel1DOF_YRecord;
