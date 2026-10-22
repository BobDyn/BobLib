within BobLib.Chassis.Interfaces;
expandable connector ChassisBus

  "Typed ChassisBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.ChassisBus;

  SI.Velocity longitudinalVelocity "Chassis: signed forward velocity";
  SI.Velocity vehicleSpeed "Chassis: nonnegative magnitude of body velocity";
  SI.Acceleration bodyAcceleration_2 "Chassis: lateral body acceleration";
  SI.Length rideHeight_1 "Chassis: ride height at corner 1";
  SI.Length rideHeight_2 "Chassis: ride height at corner 2";
  SI.Length rideHeight_3 "Chassis: ride height at corner 3";
  SI.Length rideHeight_4 "Chassis: ride height at corner 4";
  SI.Length frontLeftSpringLength "Front axle: left spring/damper length";
  SI.Length frontRightSpringLength "Front axle: right spring/damper length";
  SI.Length rearLeftSpringLength "Rear axle: left spring/damper length";
  SI.Length rearRightSpringLength "Rear axle: right spring/damper length";

  SI.Force Fz_1 "Chassis: tire normal load at corner 1";
  SI.Force Fz_2 "Chassis: tire normal load at corner 2";
  SI.Force Fz_3 "Chassis: tire normal load at corner 3";
  SI.Force Fz_4 "Chassis: tire normal load at corner 4";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end ChassisBus;
