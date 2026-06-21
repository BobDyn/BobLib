within BobLib.Resources.Icons;

partial model BatteryPackIcon "Reusable battery pack icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -60}, {100, 60}}), graphics = {
      Rectangle(extent = {{-74, 34}, {74, -34}}, lineColor = {32, 32, 32}, fillColor = {235, 245, 239}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{74, 16}, {88, -16}}, lineColor = {32, 32, 32}, fillColor = {235, 245, 239}, fillPattern = FillPattern.Solid),
      Line(points = {{-48, 0}, {-22, 0}}, color = {40, 120, 70}, thickness = 1),
      Line(points = {{-35, 13}, {-35, -13}}, color = {40, 120, 70}, thickness = 1),
      Line(points = {{18, 0}, {48, 0}}, color = {40, 120, 70}, thickness = 1),
      Text(extent = {{-62, 54}, {62, 34}}, textString = "%name", lineColor = {32, 32, 32})}));
end BatteryPackIcon;
