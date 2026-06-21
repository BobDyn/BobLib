within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.SteeringRack;

record RackAndPinionRecord

  import SI = Modelica.Units.SI;

  parameter SI.Position leftPickup[3]
    "Left pickup coordinate, expressed in chassis frame";

  parameter Real cFactor
    "Rack C-factor (m rack travel per pinion revolution)";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>RackAndPinionRecord</code> stores steering-rack geometry and conversion data.
</p>
<p>
It maps steering input rotation to rack translation and tie-rod motion in the double-wishbone axle models.
</p>
</html>"));
end RackAndPinionRecord;
