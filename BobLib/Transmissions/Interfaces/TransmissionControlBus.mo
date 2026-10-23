within BobLib.Transmissions.Interfaces;
expandable connector TransmissionControlBus

  "Typed TransmissionControlBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.TransmissionControlBus;

  Integer currentGear "FixedRatioTransmission: engaged gear, always one";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end TransmissionControlBus;
