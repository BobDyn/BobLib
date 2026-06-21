within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire;

record Wheel1DOF_ZRecord

  import SI = Modelica.Units.SI;

  // Rate properties
  parameter SI.TranslationalSpringConstant wheelC "Wheel vertical stiffness" annotation(
    Dialog(group = "Rate Properties"));
  parameter SI.TranslationalDampingConstant wheelD "Wheel vertical damping" annotation(
    Dialog(group = "Rate Properties"));

  annotation(
    Documentation(info = "<html>
<p>
Record <code>Wheel1DOF_ZRecord</code> stores parameters for the corresponding wheel degree-of-freedom model.
</p>
<p>
It keeps wheel inertia, vertical stiffness, damping, and related values separate from the tire force equations.
</p>
</html>"));
end Wheel1DOF_ZRecord;
