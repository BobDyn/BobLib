within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip;

record MyPureRecord

  // Rolling resistance coefficients
  Real QSY1 "Base rolling resistance coefficient";
  Real QSY2 "Longitudinal force influence on rolling resistance";
  Real QSY3 "Linear speed dependency of rolling resistance";
  Real QSY4 "Quartic speed dependency of rolling resistance";

  // Reference velocity
  Real Vref "Reference longitudinal velocity for normalization";

  // Scaling
  Real LMY "Scaling factor for rolling resistance moment";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>MyPureRecord</code> stores MF5.2 pure-slip coefficients for rolling-resistance moment.
</p>
<p>
It mirrors the coefficient grouping used by the tire evaluator so each force or moment contribution can be maintained independently.
</p>
</html>"));
end MyPureRecord;
