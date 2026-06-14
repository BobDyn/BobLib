within ;
package BobLibVehicleInterfaces
  "Standalone vehicle library built from VehicleInterfaces contracts"

  annotation(
    preferredView = "info",
    version = "0.1.0",
    uses(
      Modelica(version = "4.1.0"),
      VehicleInterfaces(version = "2.0.2")),
    Icon(
      coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}),
      graphics = {
        Rectangle(lineColor = {78, 161, 255}, fillColor = {14, 17, 22}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 22),
        Polygon(points = {{-82, 74}, {-42, 74}, {-28, 90}, {82, 90}, {82, 74}, {-82, 74}}, lineColor = {24, 30, 42}, fillColor = {24, 30, 42}, fillPattern = FillPattern.Solid),
        Rectangle(lineColor = {107, 182, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-82, 72}, {82, -76}}, radius = 12),
        Text(extent = {{-76, 88}, {76, 74}}, textString = "BobLib", textColor = {107, 182, 255}, fontName = "DejaVu Sans"),
        Line(points = {{-70, -62}, {70, -62}}, color = {78, 161, 255}, thickness = 1.25),
        Line(points = {{-70, 58}, {70, 58}}, color = {230, 235, 242}),
        Bitmap(extent = {{-74, -76}, {74, 72}}, fileName = "modelica://BobLibVehicleInterfaces/Resources/Images/bobdyn.png")}),
    Documentation(info = "<html>
<p style=\"text-align:center;\">
<img src=\"modelica://BobLibVehicleInterfaces/Resources/Images/bobdyn.png\" width=\"240\"/>
</p>
<p>
BobLibVehicleInterfaces is a standalone vehicle library that starts from the
standard contracts in VehicleInterfaces and builds BobLib components inside
those subsystem boundaries.
</p>
<p>
The public package layout mirrors the VehicleInterfaces style: vehicle-level
runnable models live in <code>Experiments</code>, reusable drawings live in
<code>Icons</code>, and vehicle parameters live in <code>Records</code>.
Direct subsystem models in top-level domains form the shared-contract layer:
they extend or adapt the VehicleInterfaces subsystem contracts and are the
models redeclared into the vehicle stack. Detailed BobLib physics, templates,
mounts, and helper functions live in nested packages below those domains.
</p>
<p>
Subsystem domains include <code>Chassis</code>, <code>Drivelines</code>,
<code>EnergyStorage</code>, <code>ElectricDrives</code>,
<code>Controllers</code>, <code>PowerElectronics</code>, and
<code>Aero</code>. Regression and component tests are kept in the sibling
library <code>BobLibVehicleInterfacesTests</code>. Shared mechanics helpers are
kept under <code>Utilities.Mechanics</code>.
</p>
</html>"));
end BobLibVehicleInterfaces;
