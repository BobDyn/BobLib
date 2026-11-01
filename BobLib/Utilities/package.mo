within BobLib;
package Utilities

  "General math and mechanics helpers that do not map onto VehicleInterfaces roots"
  extends Modelica.Icons.UtilitiesPackage;

  annotation(uses(Modelica(version = "4.1.0")),
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities</code> contains shared utilities that do not map to a VehicleInterfaces subsystem root.
</p>
<p>
Math and mechanics helpers are kept here so domain packages can stay focused on vehicle physics and adapters.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Utilities;
