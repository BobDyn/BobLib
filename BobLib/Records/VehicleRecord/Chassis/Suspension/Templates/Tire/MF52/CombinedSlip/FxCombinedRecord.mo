within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

record FxCombinedRecord

  // Reduction (Gx)
  Real RBX1 "Slope factor for combined slip reduction [-]";
  Real RBX2 "Slip influence on combined reduction [-]";
  Real RCX1 "Shape factor of reduction function [-]";
  Real REX1 "Curvature base for reduction [-]";
  Real REX2 "Load dependency of reduction curvature [-]";

  // Shift (Shxa)
  Real RHX1 "Slip angle shift for combined slip [-]";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>FxCombinedRecord</code> stores MF5.2 combined-slip coefficients for longitudinal force.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end FxCombinedRecord;
