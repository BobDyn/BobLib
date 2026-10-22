within BobLib.Chassis.Suspension;
package Tires

  "Tire, wheel, slip, and contact-patch frame models used by suspension axles"

  // Package graphic adapted from BobLib.Icons.BaseTireIcon; instance-name text omitted.

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {

      Rectangle(fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-40, 80}, {40, -80}}, radius = 5),
      Line(points = {{-22, -84}, {22, -84}}, color = {20, 20, 20}, thickness = 1, arrow = {Arrow.Filled, Arrow.Filled}),
      Line(points = {{0, -98}, {0, -72}}, color = {20, 20, 20}, thickness = 1, arrow = {Arrow.Filled, Arrow.Filled})

    , Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Chassis.Suspension.Tires</code> contains tire models under the suspension domain.
</p>
<p>
Tires live with suspension assemblies because axle models own wheel centers,
tire load paths, and raw contact-patch frames. Shared MultiBody contact
mechanics, fixtures, and actuators live under
<code>Utilities.Mechanics.MultiBody</code> and are connected at the chassis or
rig assembly level.
</p>
</html>"));
end Tires;
