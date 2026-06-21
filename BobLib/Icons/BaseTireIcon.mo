within BobLib.Icons;

partial model BaseTireIcon "Reusable base tire icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-40, 80}, {40, -80}}, radius = 5),
      Line(points = {{-22, -84}, {22, -84}}, color = {20, 20, 20}, thickness = 1, arrow = {Arrow.Filled, Arrow.Filled}),
      Line(points = {{0, -98}, {0, -72}}, color = {20, 20, 20}, thickness = 1, arrow = {Arrow.Filled, Arrow.Filled})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>BaseTireIcon</code> defines shared icon geometry for base tire icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end BaseTireIcon;
