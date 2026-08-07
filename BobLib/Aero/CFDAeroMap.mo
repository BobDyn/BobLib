within BobLib.Aero;

model CFDAeroMap "CFD-based aero load map from per-corner ride heights"

  extends BobLib.Aero.Interfaces.Base;
  extends BobLib.Icons.CFDAeroMapIcon;

  import SI = Modelica.Units.SI;
  import BobLib.Aero.Internal.Bilinear2D;
  parameter BobLib.Records.VehicleRecord.Aero.CFDAeroMapRecord pAero "CFD aero map record";
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
  constant SI.Velocity flowDirectionSpeedEps = 1e-6
    "Lower bound used only when normalizing relative airflow";
  SI.Velocity flowDirectionSpeed;
  Real relativeAirDirection[3]
    "Vehicle-relative air velocity direction resolved in the body frame";

equation
  assert(pAero.referenceSpeed > 0, "CFDAeroMap: referenceSpeed must be positive");
  assert(pAero.referenceDensity > 0, "CFDAeroMap: referenceDensity must be positive");

  frontRideHeight = (rideHeight_1 + rideHeight_2) / 2;
  rearRideHeight = (rideHeight_3 + rideHeight_4) / 2;

  speedScale =
    noEvent(max(airDensity, 0)/pAero.referenceDensity)*
    (relativeAirSpeed/pAero.referenceSpeed)*
    (relativeAirSpeed/pAero.referenceSpeed);
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

  flowDirectionSpeed = noEvent(max(relativeAirSpeed, flowDirectionSpeedEps));
  relativeAirDirection = relativeAirVelocity/flowDirectionSpeed;

  // Body-frame convention: x forward, z up. The drag table supplies a
  // positive magnitude, while its direction opposes vehicle-relative airflow.
  // Downforce and moments retain the body axes used to calibrate the CFD map.
  force = -drag*relativeAirDirection + {0, 0, -downforce};
  torque = {speedScale*mxRaw, speedScale*myRaw, speedScale*mzRaw};
  annotation(
    Diagram,
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
The CFD tables use the forward-flow, body-axis convention in which
<code>dragTable</code> stores a positive drag magnitude. That magnitude is
applied opposite the full vehicle-relative air-velocity vector, so reverse
motion, following wind, and crosswind produce a correspondingly directed drag
force. The tabulated downforce and moments remain resolved in the body axes to
preserve their calibration convention. Using this map for large flow angles is
therefore a directional extrapolation of forward-flow CFD data rather than a
replacement for yaw- and pitch-swept aerodynamic maps.
</p>
<p>
It is part of the aerodynamic load path. Aero models receive chassis-owned
ride-height measurements through the shared VehicleInterfaces control bus and
atmosphere-owned measurements through the BobLib atmosphere bus, then provide
body-frame force and torque outputs for the chassis.
</p>
</html>"));
end CFDAeroMap;
