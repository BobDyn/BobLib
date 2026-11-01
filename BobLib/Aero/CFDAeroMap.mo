within BobLib.Aero;

model CFDAeroMap "CFD-based aero load map from per-corner ride heights"

  import Modelica.Units.SI;
  import BobLib.Aero.Internal.Bilinear2D;

  extends BobLib.Aero.Interfaces.Base;
  extends BobLib.Icons.CFDAeroMapIcon;

  parameter BobLib.Records.VehicleRecord.Aero.CFDAeroMapRecord pAero "Aerodynamic forces, moments, ride-height grids and reference-condition calibration" annotation(Dialog(group = "Calibration"));
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
    pAero.dragTable, pAero.frontPoints, pAero.rearPoints);
  downforceRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.downforceTable, pAero.frontPoints, pAero.rearPoints);
  mxRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.mxTable, pAero.frontPoints, pAero.rearPoints);
  myRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.myTable, pAero.frontPoints, pAero.rearPoints);
  mzRaw = Bilinear2D(
    frontRideHeight,
    rearRideHeight,
    pAero.frontRideHeightGrid,
    pAero.rearRideHeightGrid,
    pAero.mzTable, pAero.frontPoints, pAero.rearPoints);
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
aero load-point velocity and the local wind/density evaluated through VI atmosphere functions.
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
local density and wind through the shared VI atmosphere functions, then provide
body-frame force and torque outputs for the chassis.
</p>
</html>"));
end CFDAeroMap;
