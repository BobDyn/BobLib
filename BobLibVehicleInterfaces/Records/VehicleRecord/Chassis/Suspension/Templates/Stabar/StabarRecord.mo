within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Stabar;

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

  annotation(
    Documentation(info = "<html>
<p>
Record <code>StabarRecord</code> stores stabilizer-bar geometry and rate data.
</p>
<p>
Axle variants with anti-roll bars use this record to connect left and right suspension motion through the bar linkage.
</p>
</html>"));
end StabarRecord;
