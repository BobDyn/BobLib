within BobLibVehicleInterfaces.Records.VehicleRecord.Chassis;
package Suspension

  extends BobLibVehicleInterfaces.Icons.RecordsPackageIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension</code>
contains suspension and axle assembly records.
</p>
<p>
Axle records live at this level because the public suspension axle models are
inserted from this level of the chassis subsystem. Linkage, steering, tire, and
mass-property records live under <code>Templates</code>.
</p>
</html>"));
end Suspension;
