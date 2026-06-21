within BobLibVehicleInterfaces.Icons;

partial package FMIIcon

  "Reusable package icon for FMI utilities"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageFrame;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {
      Rectangle(lineColor = {14, 17, 22}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-62, 44}, {62, -44}}, radius = 8),
      Rectangle(lineColor = {14, 17, 22}, fillColor = {230, 235, 242}, fillPattern = FillPattern.Solid, extent = {{-44, 24}, {-10, -24}}, radius = 4),
      Rectangle(lineColor = {14, 17, 22}, fillColor = {230, 235, 242}, fillPattern = FillPattern.Solid, extent = {{12, 24}, {46, -24}}, radius = 4),
      Line(points = {{-10, 0}, {12, 0}}, color = {78, 161, 255}, thickness = 1.5, arrow = {Arrow.None, Arrow.Filled}),
      Text(extent = {{-72, -76}, {72, -48}}, textString = "FMI", textColor = {14, 17, 22}, fontName = "DejaVu Sans")}),
    Documentation(info = "<html>
<p>
Partial package <code>FMIIcon</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end FMIIcon;
