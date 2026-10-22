within BobLib.Aero;
package Mounts

  "Aerodynamic load-frame mount adapters"

  // Package graphic adapted from BobLib.Aero.Mounts.RigidMount; instance-name text omitted.

  annotation(Icon(coordinateSystem(extent = {{-120, -80}, {120, 80}}), graphics = {
      Rectangle(extent = {{-80, 20}, {80, -20}}, lineColor = {95, 95, 95}, fillColor = {235, 235, 235}, fillPattern = FillPattern.Solid),
      Line(points = {{-100, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 1), Rectangle(extent = {{-120, 80}, {120, -80}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Aero.Mounts</code> contains aerodynamic load mounting models.
</p>
<p>
Mounts translate aero force and torque outputs into chassis-frame load application points while keeping the map model independent of chassis geometry.
</p>
</html>"));
end Mounts;
