within BobLibVehicleInterfaces.Aero;

model CFDAeroMap "CFD-based aero load map from per-corner ride heights"
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
  SI.Length frontRideHeight;
  SI.Length rearRideHeight;
equation
  assert(pAero.referenceSpeed > 0, "CFDAeroMap: referenceSpeed must be positive");
  assert(pAero.referenceDensity > 0, "CFDAeroMap: referenceDensity must be positive");

  frontRideHeight = (rideHeight_1 + rideHeight_2) / 2;
  rearRideHeight = (rideHeight_3 + rideHeight_4) / 2;

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
Model <code>CFDAeroMap</code>: CFD-based aero load map from per-corner ride heights.
</p>
<p>
The model reads front-left, front-right, rear-left, and rear-right ride-height
signals from <code>controlBus.chassisBus</code>. It averages each axle
internally before looking up the front/rear ride-height CFD tables, then scales
the loads with dynamic pressure using relative airspeed calculated from the
sprung chassis frame and wind plus local air density read from
<code>atmosphereBus</code>.
</p>
<p>
It is part of the aerodynamic load path. Aero models receive chassis-owned
ride-height measurements through the shared VehicleInterfaces control bus and
atmosphere-owned measurements through the BobLib atmosphere bus, then provide
body-frame force and torque outputs for the chassis.
</p>
</html>"));
end CFDAeroMap;
