within BobLib.Resources.Icons;

partial model PowertrainBatInvMotDiffIcon "Reusable powertrain bat inv mot diff icon"
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-86, 56}, {-44, 18}}, lineColor = {35, 35, 35}, fillColor = {235, 245, 239}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-34, 56}, {8, 18}}, lineColor = {35, 35, 35}, fillColor = {240, 243, 248}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{18, 50}, {58, 10}}, lineColor = {35, 35, 35}, fillColor = {230, 235, 240}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{36, -14}, {76, -54}}, lineColor = {35, 35, 35}, fillColor = {238, 238, 238}, fillPattern = FillPattern.Solid),
      Line(points = {{-44, 36}, {-34, 36}}, color = {0, 0, 255}),
      Line(points = {{8, 36}, {18, 30}}, color = {0, 0, 127}),
      Line(points = {{58, 30}, {56, -14}}, color = {95, 95, 95}, thickness = 1),
      Text(extent = {{-90, 88}, {90, 62}}, textString = "%name", lineColor = {32, 32, 32})
    }));
end PowertrainBatInvMotDiffIcon;
