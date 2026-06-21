within BobLibVehicleInterfaces.DriverEnvironments;

package Internal

  extends BobLibVehicleInterfaces.Icons.BobLibInternalPackageBackground;
  annotation(
    Documentation(info = "<html>
<p>
Internal driver-environment building blocks. Public first-level driver
environment adapters compose these small publishers to preserve the
VehicleInterfaces driver-environment boundary while keeping transmission,
ignition, EV, and IC-specific signals out of the core driver intent.
</p>
</html>"));
end Internal;
