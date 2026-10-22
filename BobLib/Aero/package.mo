within BobLib;
package Aero

  "Aerodynamic interfaces and chassis mount adapters"

  // Package graphic adapted from BobLib.Aero.Interfaces.Base; instance-name text omitted.

  annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}}), graphics = {
      Rectangle(extent = {{-140, 60}, {140, -60}}, lineColor = {95, 95, 95}, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, radius = 8),
      Line(points = {{-120, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 1),
      Polygon(points = {{-25, 0}, {30, 30}, {95, 30}, {70, 0}, {95, -30}, {30, -30}, {-25, 0}}, lineColor = {0, 85, 170}, fillColor = {210, 230, 255}, fillPattern = FillPattern.Solid), Rectangle(extent = {{-160, 100}, {160, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
This package defines the public aerodynamic subsystem contract used by the
vehicle assemblies. The aero interface is BobLib-specific, but it follows the
VehicleInterfaces pattern: a small public subsystem boundary, one direct
experiment-facing subsystem model, and helper implementations nested below the
domain package.
</p>
</html>"));
end Aero;
