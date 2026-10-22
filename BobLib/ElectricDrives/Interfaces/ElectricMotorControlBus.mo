within BobLib.ElectricDrives.Interfaces;
expandable connector ElectricMotorControlBus

  "Typed ElectricMotorControlBus contract for BobLib subsystem signals"

  import Modelica.Units.SI;

  extends VehicleInterfaces.Interfaces.ElectricMotorControlBus;

  SI.Power availablePower "InverterDC: signed limited allocation consumed by the averaged motor";
  SI.Power powerRequest "VCU: signed requested electrical power sent to inverter";
  SI.Torque limitedTorqueCommand "VCU: limited signed torque command";
  SI.Torque regenTorqueLimit "VCU: regenerative torque limit";
  Boolean vcuActive "VCU: controller active state";
  SI.Torque driverTorqueCommand "EVDriveByWire: driver torque request; not consumed by the default averaged drive";
  SI.Torque driverRegenTorqueLimit "EVDriveByWire: driver regenerative torque limit; not consumed by the default averaged drive";

  annotation(Documentation(info = "<html><p>Actual expandable subsystem bus. Declare every new BobLib signal here with its type, units and description before connecting it. Connections select the members present in an architecture; each signal has one publisher. Corner numbers are 1 front-left, 2 front-right, 3 rear-left, 4 rear-right. Compatible with the corresponding empty VehicleInterfaces bus.</p></html>"));
end ElectricMotorControlBus;
