within BobLib.Experiments;
package Standards "Standard vehicle, FMI, and four-post experiments"

  extends Modelica.Icons.ExamplesPackage;
  annotation(Documentation(info = "<html><p>Start with VehicleSim, VehicleFMI, or FourPostSim. BaseClasses holds the shared vehicle plant and experiment implementations. Examples demonstrates independent front/rear choices. Exhaustive configuration coverage belongs to the sibling Tests library.</p></html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Standards;
