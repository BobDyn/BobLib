within BobLib.Icons;

partial model CFDAeroMapIcon "Reusable CFD aero map icon overlay"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(
        extent = {{-62, -54}, {62, -82}},
        lineColor = {0, 85, 170},
        fillColor = {235, 244, 255},
        fillPattern = FillPattern.Solid,
        radius = 4),
      Line(points = {{-48, -76}, {-48, -60}}, color = {120, 145, 170}),
      Line(points = {{-24, -76}, {-24, -60}}, color = {120, 145, 170}),
      Line(points = {{0, -76}, {0, -60}}, color = {120, 145, 170}),
      Line(points = {{24, -76}, {24, -60}}, color = {120, 145, 170}),
      Line(points = {{48, -76}, {48, -60}}, color = {120, 145, 170}),
      Line(points = {{-56, -70}, {56, -70}}, color = {120, 145, 170}),
      Line(points = {{-56, -64}, {56, -64}}, color = {120, 145, 170}),
      Line(
        points = {{-52, -76}, {-26, -68}, {-4, -72}, {20, -61}, {50, -64}},
        color = {0, 85, 170},
        thickness = 0.75),
      Text(
        extent = {{-56, -52}, {56, -32}},
        textString = "CFD",
        textColor = {0, 85, 170})}),
    Documentation(info = "<html>
<p>
Partial model <code>CFDAeroMapIcon</code> defines the CFD map overlay used by
aero map models. The base aero interface owns the body and load-path icon; this
partial adds only the map cue.
</p>
<p>
It contains only graphical annotation primitives. Component and package models
extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end CFDAeroMapIcon;
