within BobLib.Resources.Icons;

partial model DifferentialIcon "Reusable differential icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Ellipse(extent = {{-42, 42}, {42, -42}}, lineColor = {45, 45, 45}, fillColor = {238, 238, 238}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-16, 16}, {16, -16}}, lineColor = {45, 45, 45}, fillColor = {210, 215, 220}, fillPattern = FillPattern.Solid),
      Line(points = {{-28, 0}, {28, 0}}, color = {45, 45, 45}, thickness = 1),
      Line(points = {{0, -28}, {0, 28}}, color = {45, 45, 45}, thickness = 1),
      Text(extent = {{-70, 78}, {70, 50}}, textString = "%name", lineColor = {32, 32, 32})
    }));
end DifferentialIcon;
