within BobLib.Chassis;
package Suspension

  "Suspension, axle, tire, and contact-patch frame physics"

  // Package graphic adapted from BobLib.Icons.AxleDWBaseIcon; instance-name text omitted.

  annotation(Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true, grid = {4, 2}), graphics = {

      Line(origin = {-25, 35}, points = {{-35, -15}, {25, 15}}),
      Line(origin = {-30, 65}, points = {{-30, -9}, {30, -15}}),
      Line(origin = {-82, 24}, points = {{22, -4}, {-24, 4}}, thickness = 5),
      Line(origin = {-82, 60}, points = {{22, -4}, {-22, 6}}, thickness = 5),
      Ellipse(origin = {-60, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {-60, 56}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Line(origin = {-30, 65}, points = {{90, -9}, {30, -15}}),
      Line(origin = {35, -5}, points = {{-35, 55}, {25, 25}}),
      Line(origin = {84, 16}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2),
      Line(origin = {81, 61}, points = {{-21, -5}, {23, 5}}, thickness = 5),
      Ellipse(origin = {60, 56}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Ellipse(origin = {60, 20}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Line(origin = {-80, 36}, points = {{20, -6}, {-22, 6}}, thickness = 5),
      Ellipse(origin = {-60, 30}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Rectangle(origin = {-120, 50}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5),
      Line(origin = {40, 36}, points = {{20, -6}, {64, 6}}, thickness = 5),
      Ellipse(origin = {60, 30}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}),
      Rectangle(origin = {120, 50}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5),
      Line(origin = {-25, 35}, points = {{-35, -15}, {25, 15}}),
      Line(origin = {35, -5}, points = {{-35, 55}, {25, 25}}),
      Line(origin = {-10, 30}, points = {{-36, 0}, {56, 0}}, thickness = 8),
      Line(origin = {-50, 30}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5),
      Line(origin = {50, 30}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5)

    , Rectangle(extent = {{-180, 140}, {180, -20}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(Documentation(info = "<html>
<p>
Suspension-domain physics for the BobLib chassis adapter. Choose a concrete
axle directly in this package. <code>Interfaces</code> defines the shared axle
contract, <code>BaseClasses</code> supplies partial implementations intended
for extension, and <code>Components</code> provides reusable mechanisms.
Tire, wheel, slip, and contact-patch models live in <code>Tires</code> because
they are owned by the suspension/axle loop rather than by a separate
VehicleInterfaces subsystem contract. Reusable MultiBody actuators, fixtures,
and contact mechanics that close contact-patch frames to rig components live in
<code>Utilities.Mechanics.MultiBody.ContactMechanics</code>.
</p>
</html>"));
end Suspension;
