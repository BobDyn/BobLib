within BobLib;
package Experiments

  "Runnable experiments, examples, and vehicle-level simulation stacks"
  extends Modelica.Icons.ExamplesPackage;

  annotation(Documentation(info = "<html>
<p>
Vehicle-level experiments live here. Experiment models assemble public
subsystem models from the top-level domain packages, following the same
replaceable-subsystem style used by VehicleInterfaces examples.
</p>
</html>"));
  annotation(Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));
end Experiments;
