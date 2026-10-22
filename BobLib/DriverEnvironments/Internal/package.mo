within BobLib.DriverEnvironments;

package Internal

  extends Modelica.Icons.InternalPackage;
  annotation(
    Documentation(info = "<html>
<p>
Internal driver-environment building blocks. Public first-level driver
environment adapters compose these small publishers to preserve the
VehicleInterfaces driver-environment boundary while keeping transmission,
ignition, EV, and IC-specific signals out of the core driver intent.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Internal;
