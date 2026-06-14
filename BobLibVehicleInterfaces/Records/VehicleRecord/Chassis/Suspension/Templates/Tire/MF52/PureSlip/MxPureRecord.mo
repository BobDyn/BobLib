within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip;

record MxPureRecord
  // Base overturning moment
  Real QSX1 "Base overturning moment coefficient";

  // Camber influence
  Real QSX2 "Camber-induced overturning moment coefficient";

  // Lateral force coupling
  Real QSX3 "Lateral force contribution to overturning moment";

  // Scaling factors
  Real LMX  "Scaling factor for overturning moment";
  Real LVMX "Scaling factor for vertical-force-induced overturning moment";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>MxPureRecord</code> stores MF5.2 pure-slip coefficients for overturning moment.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end MxPureRecord;