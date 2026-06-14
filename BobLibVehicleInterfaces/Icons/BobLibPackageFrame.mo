within BobLibVehicleInterfaces.Icons;

partial package BobLibPackageFrame
  "BobDocs-themed package frame for BobLib-owned content"

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {78, 161, 255}, fillColor = {14, 17, 22}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 22),
      Polygon(points = {{-82, 74}, {-42, 74}, {-28, 90}, {82, 90}, {82, 74}, {-82, 74}}, lineColor = {24, 30, 42}, fillColor = {24, 30, 42}, fillPattern = FillPattern.Solid),
      Rectangle(lineColor = {107, 182, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-82, 72}, {82, -76}}, radius = 12),
      Text(extent = {{-76, 88}, {76, 74}}, textString = "BobLib", textColor = {107, 182, 255}, fontName = "DejaVu Sans"),
      Line(points = {{-70, -62}, {70, -62}}, color = {78, 161, 255}, thickness = 1.25),
      Line(points = {{-70, 58}, {70, 58}}, color = {230, 235, 242})}),
    Documentation(info = "<html>
<p>
Partial package <code>BobLibPackageFrame</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end BobLibPackageFrame;
