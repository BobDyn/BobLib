within BobLib.Utilities;
package Mechanics

  "Mechanical helper functions and MultiBody utilities"
  extends Modelica.Icons.UtilitiesPackage;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<p>
BobLib mechanics helpers live here as reusable utility
functions and frame-based utilities. Public mechanical assemblies belong in the
owning domain packages.
</p>
<p>
General mechanics calculations live under <code>Functions</code>. Reusable
MultiBody helpers that are shared by chassis-level assemblies live under
<code>MultiBody</code>.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Mechanics;
