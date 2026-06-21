within BobLib.Records;
package VehicleRecord

  extends BobLib.Icons.RecordsPackageIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Records.VehicleRecord</code> contains structured parameter records.
</p>
<p>
The first-level record packages mirror the public vehicle subsystem packages,
while deeper packages hold BobLib-specific physics records such as suspension
geometry, tire coefficients, and mass properties.
</p>
</html>"));
end VehicleRecord;
