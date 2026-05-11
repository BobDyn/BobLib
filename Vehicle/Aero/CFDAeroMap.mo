within BobLib.Vehicle.Aero;

model CFDAeroMap "CFD-based aero load map from front/rear ride height"
  import Modelica.SIunits;
  parameter BobLib.Resources.VehicleRecord.Aero.CFDAeroMapRecord pAero "CFD aero map record";
  input SIunits.Length frontRideHeight "Front ride height, meters";
  input SIunits.Length rearRideHeight "Rear ride height, meters";
  input SIunits.Velocity speed "Vehicle speed";
  output SIunits.Force force[3] "Aero force in body coordinates";
  output SIunits.Torque torque[3] "Aero torque in body coordinates";
  output SIunits.Force drag "Positive drag magnitude";
  output SIunits.Force downforce "Positive downforce magnitude";
protected
  Real speedScale;
  Real dragRaw;
  Real downforceRaw;
  Real mxRaw;
  Real myRaw;
  Real mzRaw;
equation
  speedScale = (speed/pAero.referenceSpeed)*(speed/pAero.referenceSpeed);
  dragRaw = Bilinear2D(frontRideHeight, rearRideHeight, pAero.frontRideHeightGrid, pAero.rearRideHeightGrid, pAero.dragTable);
  downforceRaw = Bilinear2D(frontRideHeight, rearRideHeight, pAero.frontRideHeightGrid, pAero.rearRideHeightGrid, pAero.downforceTable);
  mxRaw = Bilinear2D(frontRideHeight, rearRideHeight, pAero.frontRideHeightGrid, pAero.rearRideHeightGrid, pAero.mxTable);
  myRaw = Bilinear2D(frontRideHeight, rearRideHeight, pAero.frontRideHeightGrid, pAero.rearRideHeightGrid, pAero.myTable);
  mzRaw = Bilinear2D(frontRideHeight, rearRideHeight, pAero.frontRideHeightGrid, pAero.rearRideHeightGrid, pAero.mzTable);
  drag = speedScale*dragRaw;
  downforce = speedScale*downforceRaw;
// Body-frame convention: x forward, z up.
  force = {-drag, 0, -downforce};
  torque = {speedScale*mxRaw, speedScale*myRaw, speedScale*mzRaw};
  annotation(
    Documentation(info = "CFD-based aero map built from the base_frh_rrh_03 sweep. The lookup is clamped to the table edges and scaled with speed^2 from the reference condition stored in the aero record."),
    Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}})),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(fillColor = {230, 230, 230}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Ellipse(fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}), Line(origin = {6, 11}, points = {{-70, -3}, {-16, -3}, {-12, 1}, {-6, 3}, {-2, 3}, {70, 3}, {70, 3}}), Line(origin = {6, -9}, points = {{-70, 3}, {-16, 3}, {-12, -1}, {-6, -3}, {70, -3}}), Line(origin = {51, -6}, points = {{-23, 0}, {23, 0}}), Line(origin = {52, 0}, points = {{-22, 0}, {22, 0}}), Line(origin = {51, 6}, points = {{-23, 0}, {23, 0}}), Line(origin = {-39, 4}, points = {{-25, 0}, {25, 0}}), Line(origin = {-39, -2}, points = {{-25, 0}, {25, 0}}), Text(origin = {0, 49}, extent = {{-40, 19}, {40, -19}}, textString = "Aero")}));
end CFDAeroMap;