within BobLibVehicleInterfaces.Icons;

partial package BobLibInternalPackageBackground

  "BobDocs-themed BobLib package background for internal packages"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageBackground;

  annotation(
    Icon(graphics = {
      Rectangle(lineColor = {78, 161, 255}, fillColor = {24, 30, 42}, fillPattern = FillPattern.Solid, extent = {{-64, 64}, {64, 38}}, radius = 7),
      Text(extent = {{-58, 59}, {58, 43}}, textString = "internal", textColor = {107, 182, 255})}),
    Documentation(info = "<html>
<p>
Partial package <code>BobLibInternalPackageBackground</code> defines a reusable BobLib icon annotation.
</p>
<p>
It belongs to the icon layer and has no equations or connectors. Models extend these partial packages to share a common visual identity without copy/pasting graphics.
</p>
</html>"));
end BobLibInternalPackageBackground;
