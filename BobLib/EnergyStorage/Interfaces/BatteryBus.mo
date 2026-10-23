within BobLib.EnergyStorage.Interfaces;
expandable connector BatteryBus

  "Typed BatteryBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.BatteryBus;

  SI.Current current "BatteryPack: discharge-positive terminal current";
  SI.Power power "BatteryPack: discharge-positive terminal power";
  Real soc(unit = "1") "BatteryPack: state of charge as a fraction";
  Real soe(unit = "1") "BatteryPack: state of energy as a fraction";
  SI.Voltage voltage "BatteryPack: positive-to-negative terminal voltage";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end BatteryBus;
