within BobLibVehicleInterfaces.Icons;

partial model ShockLinkageIcon "Reusable shock linkage icon"

  annotation(
    Icon(graphics = {
      Line(
        origin = {1, -40},
        points = {{-61, 0}, {-41, 0}, {-31, 20}, {-11, -20}, {11, 20}, {29, -20}, {39, 0}, {59, 0}},
        thickness = 3),
      Line(origin = {-60, -15}, points = {{0, -25}, {0, 55}}, thickness = 3),
      Line(origin = {-40, 40}, points = {{-20, 0}, {30, 0}}, thickness = 3),
      Line(origin = {-10, 40}, points = {{0, -28}, {0, 26}}, thickness = 3),
      Line(origin = {0, 39}, points = {{-20, 31}, {20, 31}, {20, -31}, {-20, -31}}, thickness = 3),
      Line(origin = {60, -15}, points = {{0, -25}, {0, 55}}, thickness = 3),
      Line(origin = {40, 40}, points = {{-20, 0}, {20, 0}}, thickness = 3),
      Line(
        origin = {-29.91, -67.91},
        points = {{-2.0908, 7.9092}, {39.9092, 17.9092}, {59.9092, 47.9092}},
        color = {255, 0, 0},
        thickness = 3),
      Line(
        origin = {9.96, 38.96},
        points = {{-39.9639, -30.9639}, {-9.96392, 21.0361}, {20.0361, 31.0361}},
        color = {255, 0, 0},
        thickness = 3),
      Ellipse(origin = {80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-5, 5}, {5, -5}}),
      Ellipse(origin = {-80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-5, 5}, {5, -5}})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>ShockLinkageIcon</code> defines shared icon geometry for shock linkage icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end ShockLinkageIcon;
