within BobLib.Vehicle.Aero;

model CFDAeroMap "CFD-based aero load map from per-corner ride heights"

  extends BobLib.Resources.Icons.CFDAeroMapIcon;

  import SI = Modelica.Units.SI;
  parameter BobLib.Resources.VehicleRecord.Aero.CFDAeroMapRecord pAero "CFD aero map record";
  input SI.Length rideHeight_1 "Front-left ride height, meters";
  input SI.Length rideHeight_2 "Front-right ride height, meters";
  input SI.Length rideHeight_3 "Rear-left ride height, meters";
  input SI.Length rideHeight_4 "Rear-right ride height, meters";
  input SI.Velocity speed "Vehicle speed";
  output SI.Force force[3] "Aero force in body coordinates";
  output SI.Torque torque[3] "Aero torque in body coordinates";
  output SI.Force drag "Positive drag magnitude";
  output SI.Force downforce "Positive downforce magnitude";

protected
  Real speedScale;
  Real dragRaw;
  Real downforceRaw;
  Real mxRaw;
  Real myRaw;
  Real mzRaw;
  SI.Length frontRideHeight;
  SI.Length rearRideHeight;

equation
  frontRideHeight = (rideHeight_1 + rideHeight_2) / 2;
  rearRideHeight = (rideHeight_3 + rideHeight_4) / 2;
  speedScale = (speed/pAero.referenceSpeed)*(speed/pAero.referenceSpeed);
  dragRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.dragTable);
  downforceRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.downforceTable);
  mxRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.mxTable);
  myRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.myTable);
  mzRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.mzTable);
  drag = speedScale*dragRaw;
  downforce = speedScale*downforceRaw;

// Body-frame convention: x forward, z up.
  force = {-drag, 0, -downforce};
  torque = {speedScale*mxRaw, speedScale*myRaw, speedScale*mzRaw};
  annotation(
    Documentation(info = "CFD-based aero map built from per-corner ride-height signals. The model averages front and rear ride heights internally before looking up the front/rear ride-height sweep, clamps to the table edges, and scales with speed^2 from the reference condition stored in the aero record."),
    Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}})));
end CFDAeroMap;
