within BobLib.ElectricDrives.Interfaces;
expandable connector ElectricMotorBus

  "Typed ElectricMotorBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.ElectricMotorBus;

  SI.AngularVelocity speed "Motor: signed shaft speed";
  SI.Power electricalPower "Motor: electrical power, positive when motoring";
  SI.Power mechanicalPower "Motor: delivered shaft power, positive when motoring";
  SI.Torque torqueCommand "Motor: internal commanded torque";
  SI.Torque torqueLimit "Motor: active torque limit";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end ElectricMotorBus;
