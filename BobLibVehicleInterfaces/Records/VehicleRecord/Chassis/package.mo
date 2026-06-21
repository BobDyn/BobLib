within BobLibVehicleInterfaces.Records.VehicleRecord;
package Chassis

  extends BobLibVehicleInterfaces.Icons.RecordsPackageIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Records.VehicleRecord.Chassis</code>
contains chassis-domain parameter records.
</p>
<p>
The package follows the chassis model tree: top-level records belong to the
VehicleInterfaces-facing chassis adapter, while suspension geometry, tire data,
and linkage mass properties live in nested packages.
</p>
</html>"));
end Chassis;
