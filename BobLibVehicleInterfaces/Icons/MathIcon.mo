within BobLibVehicleInterfaces.Icons;

partial package MathIcon
  "Reusable package icon for math utilities"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageFrame;

  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics = {
      Line(points = {{-62, -48}, {-62, 54}}, color = {14, 17, 22}, thickness = 1.5, arrow = {Arrow.None, Arrow.Filled}),
      Line(points = {{-72, -38}, {62, -38}}, color = {14, 17, 22}, thickness = 1.5, arrow = {Arrow.None, Arrow.Filled}),
      Line(points = {{-52, -20}, {-34, 6}, {-16, 18}, {6, 16}, {28, 2}, {52, 34}}, color = {78, 161, 255}, thickness = 2.5, smooth = Smooth.Bezier),
      Text(extent = {{-28, -70}, {72, -28}}, textString = "f(x)", textColor = {14, 17, 22}, fontName = "DejaVu Sans"),
      Text(extent = {{-54, 42}, {54, 72}}, textString = "Math", textColor = {14, 17, 22}, fontName = "DejaVu Sans")}),
    Documentation(info = "<html>
<p>
Partial package <code>MathIcon</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end MathIcon;
