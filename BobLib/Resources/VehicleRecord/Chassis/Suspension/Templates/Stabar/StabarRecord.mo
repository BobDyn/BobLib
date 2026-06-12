within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar;

record StabarRecord
  import SI = Modelica.Units.SI;

  parameter SI.Position leftBarEnd[3] "Left end of torsion bar, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position leftArmEnd[3] "Left end of arm, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.RotationalSpringConstant barRate "Torsion bar rate" annotation(
    Evaluate = false,
    Dialog(group = "Rates"));

end StabarRecord;
