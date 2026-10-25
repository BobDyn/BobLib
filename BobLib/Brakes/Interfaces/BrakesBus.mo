within BobLib.Brakes.Interfaces;
expandable connector BrakesBus

  "Typed BrakesBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.BrakesBus;

  SI.AngularVelocity wheelSpeed_1 "BrakeActuation: wheel-equivalent speed at corner 1; central brake uses differential gearing";
  SI.AngularVelocity wheelSpeed_2 "BrakeActuation: wheel-equivalent speed at corner 2; central brake uses differential gearing";
  SI.AngularVelocity wheelSpeed_3 "BrakeActuation: wheel-equivalent speed at corner 3; central brake uses differential gearing";
  SI.AngularVelocity wheelSpeed_4 "BrakeActuation: wheel-equivalent speed at corner 4; central brake uses differential gearing";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end BrakesBus;
