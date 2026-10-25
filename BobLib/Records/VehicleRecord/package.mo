within BobLib.Records;
package VehicleRecord

  extends Modelica.Icons.RecordsPackage;

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
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end VehicleRecord;
