within BobLib.DriverEnvironments.Interfaces;
expandable connector DriverBus

  "Typed DriverBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.DriverBus;

  Real acceleratorPedalPosition(unit = "1") "Driver environment: normalized accelerator position, zero to one";
  Real brakePedalPosition(unit = "1") "Driver environment: normalized brake position, zero to one";
  SI.Angle steeringWheelAngle "Driver environment: signed handwheel angle";
  Integer gearboxMode "Automatic driver environment: mode encoded using VI gearbox conventions";
  Integer ignition "Automatic driver environment: ignition state encoded using VI conventions";
  Integer requestedGear "Automatic driver environment: requested gear; interpreted according to gearbox mode";
  Boolean inverterEnable "EV driver environment or experiment: inverter enable request";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end DriverBus;
