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
EV-specific driver models may add BobLib command outputs for VCU integration,
while still publishing standard driver signals on the VehicleInterfaces
<code>driverBus</code> and using the standard steering/chassis connectors.
</p>
</html>"));
end DriverEnvironments;
