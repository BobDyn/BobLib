within BobLib.Chassis;
package Body

  // Package graphic adapted from BobLib.Icons.FrameBaseIcon; instance-name text omitted.

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true), graphics = {Rectangle(extent = {{-20, 8}, {20, -8}}, lineColor = {95, 95, 95}, pattern = LinePattern.Dash, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid),  Line(origin = {-60, 0}, points = {{-40, 0}, {40, 0}}, thickness = 5), Line(origin = {60, 0}, points = {{40, 0}, {-40, 0}}, thickness = 5), Line(origin = {-15, -20}, points = {{7, -28}, {-15, 20}}, thickness = 1), Line(origin = {15, -20}, points = {{-7, -28}, {15, 20}}, thickness = 1), Ellipse(origin = {0, -42}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 10}, {10, -10}}), Line(origin = {0, -42}, points = {{0, -10}, {0, 10}}), Line(origin = {0, -42}, points = {{-10, 0}, {10, 0}}), Polygon(origin = {5, -37}, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Polygon(origin = {-5, -47}, rotation = 180, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Line(origin = {0, -51}, points = {{0, -9}, {0, 9}}), Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Chassis.Body</code> contains sprung body frame models.
</p>
<p>
These models provide rigid or compliant chassis-frame mass and inertia behavior for the detailed chassis assembly.
</p>
</html>"));
end Body;
