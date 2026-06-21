within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

record MyCombinedRecord

  // No additional MF5.2 combined-slip terms
  // Exists for structural consistency
  annotation(
    Documentation(info = "<html>
<p>
Record <code>MyCombinedRecord</code> stores MF5.2 combined-slip coefficients for rolling-resistance moment.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end MyCombinedRecord;