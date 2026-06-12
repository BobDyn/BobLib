within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire;

record Wheel1DOF_YRecord
  import SI = Modelica.Units.SI;

  // Mass properties
  parameter SI.Inertia wheelJ "Effective inertia of rotating mass" annotation(Dialog(group = "Mass Properties"));

end Wheel1DOF_YRecord;
