within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates;
package Tire

  extends Modelica.Icons.RecordsPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire</code>
contains tire and wheel parameter records.
</p>
<p>
Tire records remain under suspension because axle assemblies own wheel centers,
tire load paths, and raw contact-patch frames. Detailed tire-model coefficient
sets live in nested packages.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Tire;
