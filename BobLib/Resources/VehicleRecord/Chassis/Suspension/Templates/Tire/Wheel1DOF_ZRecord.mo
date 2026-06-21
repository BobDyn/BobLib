within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire;

record Wheel1DOF_ZRecord

  import SI = Modelica.Units.SI;

  // Rate properties
  parameter SI.TranslationalSpringConstant wheelC "Wheel vertical stiffness" annotation(
    Dialog(group = "Rate Properties"));
  parameter SI.TranslationalDampingConstant wheelD "Wheel vertical damping" annotation(
    Dialog(group = "Rate Properties"));

end Wheel1DOF_ZRecord;
