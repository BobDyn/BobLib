within BobLibVehicleInterfaces.Icons;

partial model SteeringWheelOverlayIcon "Reusable steering-wheel overlay for steerable axles"
  annotation(
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {
      Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}),
      Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5),
      Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5),
      Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>SteeringWheelOverlayIcon</code> defines shared icon geometry for steering-wheel overlay for steerable axles.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end SteeringWheelOverlayIcon;
