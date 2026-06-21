within BobLib.Resources.Icons;

partial model GroundPhysicsIcon "Reusable ground physics icon"

  annotation(
    Icon(
      coordinateSystem(extent = {{-100, -100}, {100, 100}}),
      graphics = {
        Text(extent = {{-120, -160}, {120, -120}}, textString = "%name", fontSize = 14,
          horizontalAlignment = TextAlignment.Center),
        Rectangle(extent = {{-100, -100}, {100, 25}}, fillPattern = FillPattern.Solid,
          fillColor = {150, 100, 50}, lineThickness = 2),
        Rectangle(extent = {{-100, 25}, {100, 100}}, fillPattern = FillPattern.Solid,
          fillColor = {95, 200, 70}, lineThickness = 2)
      }));
end GroundPhysicsIcon;
