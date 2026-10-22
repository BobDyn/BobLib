within BobLib.Icons;

partial model Bellcrank2Icon "Reusable bellcrank2 icon"

  annotation(
    Icon(graphics = {
      Text(extent = {{-150, 130}, {150, 105}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0),
      Line(points = {{-80, 0}, {0, -80}, {80, 0}, {2, 0}, {-80, 0}}, thickness = 3),
      Ellipse(origin = {-80, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-10, 10}, {10, -10}})}),
    Documentation(info = "<html>
<p>
Partial model <code>Bellcrank2Icon</code> defines shared icon geometry for bellcrank2 icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end Bellcrank2Icon;
