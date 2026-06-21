within BobLib.Records.VehicleRecord.Chassis;
package Suspension

  extends BobLib.Icons.RecordsPackageIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Records.VehicleRecord.Chassis.Suspension</code>
contains suspension and axle assembly records.
</p>
<p>
Axle records live at this level because the public suspension axle models are
inserted from this level of the chassis subsystem. Linkage, steering, tire, and
mass-property records live under <code>Templates</code>.
</p>
</html>"));
end Suspension;
