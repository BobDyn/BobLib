within BobLibVehicleInterfaces.Icons;

partial model Bellcrank2Icon "Reusable bellcrank2 icon"
  annotation(
    Icon(graphics = {
      Line(points = {{-80, 0}, {0, -80}, {80, 0}, {2, 0}, {-80, 0}}, thickness = 3),
      Ellipse(origin = {-80, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-10, 10}, {10, -10}}),
      Text(origin = {-40, -80}, extent = {{-20, -20}, {20, 20}}, textString = "P1"),
      Text(origin = {80, -40}, extent = {{-20, -20}, {20, 20}}, textString = "P2")
    }),
    Documentation(info = "<html>
<p>
Partial model <code>Bellcrank2Icon</code> defines shared icon geometry for bellcrank2 icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end Bellcrank2Icon;
