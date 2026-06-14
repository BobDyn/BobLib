within BobLibVehicleInterfaces.Icons;

partial model AxleDWBaseIcon "Reusable double-wishbone axle icon"
  annotation(
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true, grid = {4, 2}), graphics = {
      Line(origin = {-25, 35}, points = {{-35, -15}, {25, 15}}),
      Line(origin = {-30, 65}, points = {{-30, -9}, {30, -15}}),
      Line(origin = {-82, 24}, points = {{22, -4}, {-24, 4}}, thickness = 5),
      Line(origin = {-82, 60}, points = {{22, -4}, {-22, 6}}, thickness = 5),
      Ellipse(origin = {-60, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {-60, 56}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Line(origin = {-30, 65}, points = {{90, -9}, {30, -15}}),
      Line(origin = {35, -5}, points = {{-35, 55}, {25, 25}}),
      Line(origin = {84, 16}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2),
      Line(origin = {81, 61}, points = {{-21, -5}, {23, 5}}, thickness = 5),
      Ellipse(origin = {60, 56}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {60, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Line(origin = {-80, 36}, points = {{20, -6}, {-22, 6}}, thickness = 5),
      Ellipse(origin = {-60, 30}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Rectangle(origin = {-120, 50}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5),
      Line(origin = {40, 36}, points = {{20, -6}, {64, 6}}, thickness = 5),
      Ellipse(origin = {60, 30}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Rectangle(origin = {120, 50}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5),
      Line(origin = {-25, 35}, points = {{-35, -15}, {25, 15}}),
      Line(origin = {35, -5}, points = {{-35, 55}, {25, 25}}),
      Line(origin = {-10, 30}, points = {{-36, 0}, {56, 0}}, thickness = 8),
      Line(origin = {-50, 30}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5),
      Line(origin = {50, 30}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5)
    }),
    Documentation(info = "<html>
<p>
Partial model <code>AxleDWBaseIcon</code> defines shared icon geometry for double-wishbone axle icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end AxleDWBaseIcon;
