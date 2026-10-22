within BobLib;
package PowerElectronics

  "Power-electronics subsystem adapters used by VehicleInterfaces electric-drive assemblies"

  // Package graphic adapted from BobLib.Icons.InverterDCIcon; instance-name text omitted.

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {35, 35, 35}, fillColor = {240, 243, 248}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Line(points = {{-54, -24}, {-18, 24}, {18, -24}, {54, 24}}, color = {30, 90, 150}, thickness = 1), Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(Documentation(info = "<html>
<p>
Direct models in this package are vehicle-level power-electronics subsystem
boundaries. Reusable implementation details live in <code>Internal</code>, since
VehicleInterfaces 2.0.2 does not define a separate power-electronics root.
</p>
</html>"));
end PowerElectronics;
