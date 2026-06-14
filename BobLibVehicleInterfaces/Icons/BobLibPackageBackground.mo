within BobLibVehicleInterfaces.Icons;

partial package BobLibPackageBackground
  "BobDocs-themed BobLib package icon"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageFrame;

  annotation(
    Icon(graphics = {
      Text(extent = {{-70, 58}, {70, -62}}, textString = "P", textColor = {78, 161, 255}, fontName = "DejaVu Sans")}),
    Documentation(info = "<html>
<p>
Partial package <code>BobLibPackageBackground</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end BobLibPackageBackground;
