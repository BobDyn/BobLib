within BobLib;
package Icons

  "Reusable icon definitions"
  extends Modelica.Icons.IconsPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Icons</code> contains reusable icon partials and component glyphs for the standalone VehicleInterfaces-based BobLib library.
</p>
<p>
These classes only define graphical annotations. They keep the BobLib visual language centralized so models and packages can extend icons instead of duplicating drawing primitives.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Icons;
