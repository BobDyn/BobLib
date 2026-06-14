within BobLibVehicleInterfaces.Icons;

partial package RecordsPackageIcon
  "Reusable package icon for record collections"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageFrame;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {14, 17, 22}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-62, 40}, {62, -50}}, radius = 9),
      Rectangle(lineColor = {14, 17, 22}, fillColor = {255, 215, 136}, fillPattern = FillPattern.Solid, extent = {{-62, 40}, {62, 18}}, radius = 9),
      Line(points = {{-62, 8}, {62, 8}}, color = {14, 17, 22}),
      Line(points = {{-62, -20}, {62, -20}}, color = {14, 17, 22}),
      Line(points = {{-18, 40}, {-18, -50}}, color = {14, 17, 22}),
      Line(points = {{-62, -50}, {62, -50}}, color = {78, 161, 255}, thickness = 1.5)}),
    Documentation(info = "<html>
<p>
Partial package <code>RecordsPackageIcon</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end RecordsPackageIcon;
