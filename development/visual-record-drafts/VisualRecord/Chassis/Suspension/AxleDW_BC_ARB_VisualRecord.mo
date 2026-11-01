within BobLib.Records.VisualRecord.Chassis.Suspension;

record AxleDW_BC_ARB_VisualRecord

  extends BobLib.Records.VisualRecord.Chassis.Suspension.AxleDWBaseVisualRecord;

  // Left
  Real leftBellcrankPivot[3];
  Real leftBellcrankPickup1[3];
  Real leftBellcrankPickup2[3];
  Real leftBellcrankPickup3[3];
  Real leftRodMount[3];
  Real leftShockMount[3];

  Real leftBarEnd[3];
  Real leftArmEnd[3];

  // Right
  Real rightBellcrankPivot[3];
  Real rightBellcrankPickup1[3];
  Real rightBellcrankPickup2[3];
  Real rightBellcrankPickup3[3];
  Real rightRodMount[3];
  Real rightShockMount[3];

  Real rightBarEnd[3];
  Real rightArmEnd[3];

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDW_BC_ARB_VisualRecord</code> stores visualization and diagram geometry data.
</p>
<p>
It supports clean icon, diagram, and animation presentation without coupling drawing choices to the physical equations.
</p>
</html>"));
end AxleDW_BC_ARB_VisualRecord;
