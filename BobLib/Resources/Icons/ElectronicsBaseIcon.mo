within BobLib.Resources.Icons;

partial model ElectronicsBaseIcon "Reusable electronics base icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(extent = {{-78, 58}, {78, -58}}, lineColor = {35, 35, 35}, fillColor = {240, 243, 248}, fillPattern = FillPattern.Solid),
      Text(extent = {{-68, 86}, {68, 62}}, textString = "%name", lineColor = {32, 32, 32}),
      Text(extent = {{-44, 16}, {44, -16}}, textString = "ELC", lineColor = {35, 35, 35})
    }));
end ElectronicsBaseIcon;
