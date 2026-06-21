within BobLib.Icons;
partial model SimulationIcon

  "BobLib simulation model icon"
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {78, 161, 255}, fillColor = {14, 17, 22}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20),
      Polygon(points = {{-82, 74}, {-42, 74}, {-28, 90}, {82, 90}, {82, 74}, {-82, 74}}, lineColor = {24, 30, 42}, fillColor = {24, 30, 42}, fillPattern = FillPattern.Solid),
      Rectangle(lineColor = {107, 182, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-82, 72}, {82, -76}}, radius = 10),
      Text(extent = {{-76, 88}, {76, 74}}, textString = "BobLib", textColor = {107, 182, 255}, fontName = "DejaVu Sans"),
      Text(extent = {{-66, 44}, {66, -28}}, textString = "SIM", textColor = {24, 30, 42}, fontName = "DejaVu Sans"),
      Line(points = {{-58, -52}, {58, -52}}, color = {78, 161, 255}, thickness = 1.25),
      Polygon(points = {{-18, -44}, {-18, -62}, {0, -53}, {-18, -44}}, lineColor = {78, 161, 255}, fillColor = {78, 161, 255}, fillPattern = FillPattern.Solid),
      Line(points = {{8, -60}, {24, -46}, {40, -56}, {56, -40}}, color = {24, 30, 42}, thickness = 1)}),
    Documentation(info = "<html>
<p>
Partial model <code>SimulationIcon</code> defines the reusable BobLib icon for
front-facing simulation entrypoints.
</p>
</html>"));
end SimulationIcon;
