within BobLib.Drivelines.Interfaces;
expandable connector DrivelineControlBus

  "Typed DrivelineControlBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.DrivelineControlBus;

  SI.Torque driveTorqueCommand "VCU: drive torque request for diagnostics, not a second physical actuator";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end DrivelineControlBus;
