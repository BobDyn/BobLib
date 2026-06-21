within BobLib.Icons;

partial model BatteryPackIcon "Reusable battery pack icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -60}, {100, 60}}), graphics = {
      Rectangle(extent = {{-74, 34}, {74, -34}}, lineColor = {32, 32, 32}, fillColor = {235, 245, 239}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{74, 16}, {88, -16}}, lineColor = {32, 32, 32}, fillColor = {235, 245, 239}, fillPattern = FillPattern.Solid),
      Line(points = {{-48, 0}, {-22, 0}}, color = {40, 120, 70}, thickness = 1),
      Line(points = {{-35, 13}, {-35, -13}}, color = {40, 120, 70}, thickness = 1),
      Line(points = {{18, 0}, {48, 0}}, color = {40, 120, 70}, thickness = 1),
      Text(extent = {{-62, 54}, {62, 34}}, textString = "%name", lineColor = {32, 32, 32})}),
    Documentation(info = "<html>
<p>
Partial model <code>BatteryPackIcon</code> defines shared icon geometry for battery pack icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end BatteryPackIcon;
