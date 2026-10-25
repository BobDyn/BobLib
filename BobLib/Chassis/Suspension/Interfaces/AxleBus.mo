within BobLib.Chassis.Suspension.Interfaces;
expandable connector AxleBus

  "Typed AxleBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.ChassisBus;

  SI.Force leftNormalLoad "Axle: left tire normal load";
  SI.Force rightNormalLoad "Axle: right tire normal load";
  SI.Length leftSpringLength "Axle: left spring/damper length";
  SI.Length rightSpringLength "Axle: right spring/damper length";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end AxleBus;
