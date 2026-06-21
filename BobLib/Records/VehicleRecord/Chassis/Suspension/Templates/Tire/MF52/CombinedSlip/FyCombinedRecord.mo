within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip;

record FyCombinedRecord

  // Reduction (Gy)
  Real RBY1 "Slope factor for combined slip reduction [-]";
  Real RBY2 "Slip angle influence on reduction [-]";
  Real RBY3 "Shift in slip angle for reduction [-]";
  Real RCY1 "Shape factor of reduction function [-]";
  Real REY1 "Curvature base for reduction [-]";
  Real REY2 "Load dependency of reduction curvature [-]";

  // Shift (ShySR)
  Real RHY1 "Slip ratio shift base [-]";
  Real RHY2 "Slip ratio load dependency [-]";

  // Vertical shift (SvySR)
  Real RVY1 "Vertical shift base [-]";
  Real RVY2 "Vertical shift load dependency [-]";
  Real RVY3 "Camber influence on vertical shift [-]";
  Real RVY4 "Slip angle influence on vertical shift [-]";
  Real RVY5 "Slip ratio shape factor [-]";
  Real RVY6 "Slip ratio curvature factor [-]";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>FyCombinedRecord</code> stores MF5.2 combined-slip coefficients for lateral force.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end FyCombinedRecord;