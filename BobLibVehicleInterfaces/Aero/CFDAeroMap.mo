within BobLibVehicleInterfaces.Aero;

model CFDAeroMap "CFD-based aero load map from front/rear ride height"
  extends BobLibVehicleInterfaces.Aero.Interfaces.Base;
  extends BobLibVehicleInterfaces.Icons.CFDAeroMapIcon;

  import SI = Modelica.Units.SI;
  import BobLibVehicleInterfaces.Aero.Internal.Bilinear2D;
  parameter BobLibVehicleInterfaces.Records.VehicleRecord.Aero.CFDAeroMapRecord pAero "CFD aero map record";
  output SI.Force drag "Positive drag magnitude";
  output SI.Force downforce "Positive downforce magnitude";
protected
  Real speedScale;
  Real dragRaw;
  Real downforceRaw;
  Real mxRaw;
  Real myRaw;
  Real mzRaw;
equation
  assert(pAero.referenceSpeed > 0, "CFDAeroMap: referenceSpeed must be positive");
  assert(pAero.referenceDensity > 0, "CFDAeroMap: referenceDensity must be positive");

  speedScale =
    noEvent(max(airDensity, 0)/pAero.referenceDensity)*
    (relativeAirSpeed/pAero.referenceSpeed)*
    (relativeAirSpeed/pAero.referenceSpeed);
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
    Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}})),
    Documentation(info = "<html>
<p>
Model <code>CFDAeroMap</code>: CFD-based aero load map from front/rear ride height.
</p>
<p>
The lookup is clamped to the front/rear ride-height table edges and scaled
with dynamic pressure using relative airspeed and local air density from the
atmosphere model.
</p>
<p>
It is part of the aerodynamic load path. Aero models receive ride height and
atmospheric inputs, then provide body-frame force and torque outputs for the
chassis.
</p>
</html>"));
end CFDAeroMap;
