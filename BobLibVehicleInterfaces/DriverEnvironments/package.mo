within BobLibVehicleInterfaces;
package DriverEnvironments

  "Driver-environment models extending VehicleInterfaces driver contracts"
  extends Modelica.Icons.Package;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.DriverEnvironments</code> contains
driver-environment models exposed through the standard VehicleInterfaces driver
environment boundary.
</p>
<p>
The minimal driver publisher handles the architecture-neutral commands:
steering wheel, accelerator pedal, and brake pedal. Public adapters may add
architecture-specific pins only when the target vehicle actually needs them, so
EV templates do not inherit manual gear-selection or automatic gearbox-mode
commands by accident.
</p>
</html>"));
end DriverEnvironments;
