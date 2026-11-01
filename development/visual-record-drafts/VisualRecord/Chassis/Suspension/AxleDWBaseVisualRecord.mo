within BobLib.Records.VisualRecord.Chassis.Suspension;

record AxleDWBaseVisualRecord

  // Left vals

  // Wishbone (inner)
  Real leftUpperFore_i[3];
  Real leftUpperAft_i[3];
  Real leftLowerFore_i[3];
  Real leftLowerAft_i[3];

  // Upright (outer)
  Real leftUpper_o[3];
  Real leftLower_o[3];

  // Steering
  Real leftTie_i[3];
  Real leftTie_o[3];

  // Wheel
  Real leftWheelCenter[3];
  Real leftTire_ex[3];
  Real leftTire_ey[3];

  // Contact patch and loads
  Real leftCP[3];
  Real leftCPForce[3];

  // Right vals

  // Wishbone (inner)
  Real rightUpperFore_i[3];
  Real rightUpperAft_i[3];
  Real rightLowerFore_i[3];
  Real rightLowerAft_i[3];

  // Upright (outer)
  Real rightUpper_o[3];
  Real rightLower_o[3];

  // Steering
  Real rightTie_i[3];
  Real rightTie_o[3];

  // Wheel
  Real rightWheelCenter[3];
  Real rightTire_ex[3];
  Real rightTire_ey[3];

  // Contact patch and loads
  Real rightCP[3];
  Real rightCPForce[3];

  annotation(
    Documentation(info = "<html>
<p>
Record <code>AxleDWBaseVisualRecord</code> stores visualization and diagram geometry data.
</p>
<p>
It supports clean icon, diagram, and animation presentation without coupling drawing choices to the physical equations.
</p>
</html>"));
end AxleDWBaseVisualRecord;
