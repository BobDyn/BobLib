within BobLib.Aero;
package Internal

  "Internal aerodynamic helper functions"
  extends Modelica.Icons.InternalPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Aero.Internal</code> contains helper calculations used by aerodynamic models.
</p>
<p>
These helpers stay below the public aero boundary so lookup and interpolation details do not appear as subsystem insertion points.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Internal;
