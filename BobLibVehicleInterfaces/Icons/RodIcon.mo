within BobLibVehicleInterfaces.Icons;

partial model RodIcon "Reusable rod icon"
  annotation(
    Icon(graphics = {
      Line(origin = {-25.8, 3.2}, points = {{-54.2, -3.2}, {25.8, -3.2}, {105.8, -3.2}}, thickness = 5),
      Ellipse(origin = {-80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>RodIcon</code> defines shared icon geometry for rod icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end RodIcon;
