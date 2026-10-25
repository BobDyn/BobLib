within BobLib.Records.VehicleRecord.Chassis;
package Suspension

  extends Modelica.Icons.RecordsPackage;

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
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Suspension;
