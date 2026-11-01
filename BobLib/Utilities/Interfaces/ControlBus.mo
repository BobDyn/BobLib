within BobLib.Utilities.Interfaces;
expandable connector ControlBus "Typed vehicle control bus with subsystem-owned contracts"

  extends Modelica.Icons.SignalBus;

  BobLib.EnergyStorage.Interfaces.BatteryBus batteryBus "Battery bus" annotation(Dialog(enable = false));
  BobLib.Brakes.Interfaces.BrakesBus brakesBus "Brakes bus" annotation(Dialog(enable = false));
  BobLib.Brakes.Interfaces.BrakesControlBus brakesControlBus "Brakes control bus" annotation(Dialog(enable = false));
  BobLib.Chassis.Interfaces.ChassisBus chassisBus "Chassis bus" annotation(Dialog(enable = false));
  BobLib.Drivelines.Interfaces.DrivelineBus drivelineBus "Driveline bus" annotation(Dialog(enable = false));
  BobLib.Drivelines.Interfaces.DrivelineControlBus drivelineControlBus "Driveline control bus" annotation(Dialog(enable = false));
  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus "Driver bus" annotation(Dialog(enable = false));
  BobLib.ElectricDrives.Interfaces.ElectricMotorBus electricMotorBus "Electric motor bus" annotation(Dialog(enable = false));
  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus "Electric motor control bus" annotation(Dialog(enable = false));
  BobLib.Engines.Interfaces.EngineBus engineBus "Engine bus" annotation(Dialog(enable = false));
  BobLib.Transmissions.Interfaces.TransmissionBus transmissionBus "Transmission bus" annotation(Dialog(enable = false));
  BobLib.Transmissions.Interfaces.TransmissionControlBus transmissionControlBus "Transmission control bus" annotation(Dialog(enable = false));

  annotation(Documentation(info = "<html><p>Uses the VehicleInterfaces control-bus member names with BobLib typed subsystem contracts. Only implemented subsystem buses are included; every member uses a BobLib-owned contract. Used by all BobLib subsystem interfaces; can also connect to upstream VI control buses. Chassis axle measurements are flat for OpenModelica compatibility.</p></html>"));
end ControlBus;
