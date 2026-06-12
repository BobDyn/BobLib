within BobLib.Resources.Icons;

partial model Bellcrank3Icon "Reusable bellcrank3 icon"
  annotation(
    Icon(graphics = {
      Line(points = {{-80, 0}, {0, -80}, {80, 0}, {0, 80}, {-80, 0}}, thickness = 3),
      Ellipse(origin = {-80, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-10, 10}, {10, -10}}),
      Text(origin = {-40, -80}, extent = {{-20, -20}, {20, 20}}, textString = "P1"),
      Text(origin = {80, -40}, extent = {{-20, -20}, {20, 20}}, textString = "P2"),
      Text(origin = {40, 80}, extent = {{-20, -20}, {20, 20}}, textString = "P3")
    }));
end Bellcrank3Icon;
