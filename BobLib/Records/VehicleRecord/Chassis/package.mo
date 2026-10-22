within BobLib.Records.VehicleRecord;
package Chassis

  extends Modelica.Icons.RecordsPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Records.VehicleRecord.Chassis</code>
contains chassis-domain parameter records.
</p>
<p>
The package follows the chassis model tree: top-level records belong to the
VehicleInterfaces-facing chassis adapter, while suspension geometry, tire data,
and linkage mass properties live in nested packages.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Chassis;
