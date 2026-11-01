within BobLib.Utilities.Mechanics;
package Functions

  "General mechanics calculations that are not tied to MultiBody frames"
  extends Modelica.Icons.FunctionsPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities.Mechanics.Functions</code>
contains reusable mechanics calculations that operate on records, arrays, and
scalar quantities rather than on MultiBody frames.
</p>
<p>
Use this package for general mechanics math such as mass-property combination
and inertia translation. Frame-based fixtures, joints, and contact utilities
belong under <code>Utilities.Mechanics.MultiBody</code>.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Functions;
