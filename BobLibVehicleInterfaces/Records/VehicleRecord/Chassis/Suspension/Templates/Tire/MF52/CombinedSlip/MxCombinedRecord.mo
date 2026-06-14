within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

record MxCombinedRecord
  // Currently no additional parameters in MF5.2
  // Exists for architectural consistency and future extensions
  annotation(
    Documentation(info = "<html>
<p>
Record <code>MxCombinedRecord</code> stores MF5.2 combined-slip coefficients for overturning moment.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end MxCombinedRecord;