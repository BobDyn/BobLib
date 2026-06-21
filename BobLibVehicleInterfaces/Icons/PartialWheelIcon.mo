within BobLibVehicleInterfaces.Icons;

partial model PartialWheelIcon "Reusable partial wheel icon"

  annotation(
    Icon(graphics = {
      Ellipse(fillColor = {40, 40, 40}, fillPattern = FillPattern.Solid, lineThickness = 3, extent = {{-60, -60}, {60, 60}}),
      Ellipse(fillColor = {200, 200, 200}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-39, -39}, {39, 39}}),
      Ellipse(fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-15, -15}, {15, 15}}),
      Line(points = {{0, 14}, {0, 39}}, thickness = 2),
      Line(points = {{0, -14}, {0, -39}}, thickness = 2),
      Line(points = {{14, 0}, {39, 0}}, thickness = 2),
      Line(points = {{-14, 0}, {-39, 0}}, thickness = 2),
      Line(points = {{10, 10}, {28, 28}}, thickness = 2),
      Line(points = {{-10, -10}, {-28, -28}}, thickness = 2),
      Line(origin = {-2, 2}, points = {{12, -12}, {28, -30}}, thickness = 2),
      Line(points = {{-10, 10}, {-28, 28}}, thickness = 2),
      Ellipse(fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>PartialWheelIcon</code> defines shared icon geometry for partial wheel icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end PartialWheelIcon;
