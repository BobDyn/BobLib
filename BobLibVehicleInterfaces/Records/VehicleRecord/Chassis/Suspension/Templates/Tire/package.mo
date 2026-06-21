within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates;
package Tire

  extends BobLibVehicleInterfaces.Icons.RecordsPackageIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire</code>
contains tire and wheel parameter records.
</p>
<p>
Tire records remain under suspension because axle assemblies own wheel centers,
tire load paths, and raw contact-patch frames. Detailed tire-model coefficient
sets live in nested packages.
</p>
</html>"));
end Tire;
