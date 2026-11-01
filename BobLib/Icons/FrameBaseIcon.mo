within BobLib.Icons;

partial model FrameBaseIcon "Reusable frame base icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true), graphics = {Rectangle(extent = {{-20, 8}, {20, -8}}, lineColor = {95, 95, 95}, pattern = LinePattern.Dash, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid), Text(extent = {{-100, 35}, {100, 15}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0), Line(origin = {-60, 0}, points = {{-40, 0}, {40, 0}}, thickness = 5), Line(origin = {60, 0}, points = {{40, 0}, {-40, 0}}, thickness = 5), Line(origin = {-15, -20}, points = {{7, -28}, {-15, 20}}, thickness = 1), Line(origin = {15, -20}, points = {{-7, -28}, {15, 20}}, thickness = 1), Ellipse(origin = {0, -42}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 10}, {10, -10}}), Line(origin = {0, -42}, points = {{0, -10}, {0, 10}}), Line(origin = {0, -42}, points = {{-10, 0}, {10, 0}}), Polygon(origin = {5, -37}, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Polygon(origin = {-5, -47}, rotation = 180, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Line(origin = {0, -51}, points = {{0, -9}, {0, 9}})}),
    Documentation(info = "<html>
<p>
Partial model <code>FrameBaseIcon</code> defines shared icon geometry for frame base icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end FrameBaseIcon;
