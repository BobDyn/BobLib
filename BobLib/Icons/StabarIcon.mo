within BobLib.Icons;

partial model StabarIcon "Reusable stabar icon"

  annotation(
    Icon(graphics = {
      Line(origin = {0, -10}, points = {{-80, 0}, {80, 0}}, thickness = 5),
      Line(origin = {-80, 20}, points = {{0, -30}, {0, 0}}, thickness = 5),
      Line(origin = {80, 20}, points = {{0, -30}, {0, 0}}, thickness = 5),
      Ellipse(origin = {-80, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {80, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {-80, -10}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {80, -10}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Line(origin = {5, -10}, points = {{-11, 0}, {1, 0}}, color = {255, 0, 0}, thickness = 10)
    }),
    Documentation(info = "<html>
<p>
Partial model <code>StabarIcon</code> defines shared icon geometry for stabar icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end StabarIcon;
