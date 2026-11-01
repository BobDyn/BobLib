within BobLib.Utilities.Mechanics.MultiBody;
package ContactMechanics

  "MultiBody contact helpers for chassis-level contact-patch wiring"

  // Package graphic adapted from BobLib.Icons.GroundPhysicsIcon; instance-name text omitted.

  annotation(Icon(
      coordinateSystem(extent = {{-100, -100}, {100, 100}}),
      graphics = {

        Rectangle(extent = {{-100, -100}, {100, 25}}, fillPattern = FillPattern.Solid,
          fillColor = {150, 100, 50}, lineThickness = 2),
        Rectangle(extent = {{-100, 25}, {100, 100}}, fillPattern = FillPattern.Solid,
          fillColor = {95, 200, 70}, lineThickness = 2)

      , Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
Package
<code>BobLib.Utilities.Mechanics.MultiBody.ContactMechanics</code>
contains reusable contact mechanics helpers for MultiBody contact-patch frames.
</p>
<p>
Suspension and tire models expose the raw contact-patch frames. Chassis-level
assemblies use this package to close those frames to ground or fixture models,
keeping contact mechanics out of the tire implementation package.
</p>
</html>"));
end ContactMechanics;
