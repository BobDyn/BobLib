within BobLib.Resources.Icons;

partial model CFDAeroMapIcon "Reusable cfdaero map icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(fillColor = {230, 230, 230}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Ellipse(fillPattern = FillPattern.Solid, extent = {{-10, 10}, {10, -10}}), Line(origin = {6, 11}, points = {{-70, -3}, {-16, -3}, {-12, 1}, {-6, 3}, {-2, 3}, {70, 3}, {70, 3}}), Line(origin = {6, -9}, points = {{-70, 3}, {-16, 3}, {-12, -1}, {-6, -3}, {70, -3}}), Line(origin = {51, -6}, points = {{-23, 0}, {23, 0}}), Line(origin = {52, 0}, points = {{-22, 0}, {22, 0}}), Line(origin = {51, 6}, points = {{-23, 0}, {23, 0}}), Line(origin = {-39, 4}, points = {{-25, 0}, {25, 0}}), Line(origin = {-39, -2}, points = {{-25, 0}, {25, 0}}), Text(origin = {0, 49}, extent = {{-40, 19}, {40, -19}}, textString = "Aero")}));
end CFDAeroMapIcon;
