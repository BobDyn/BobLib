within ;
package BobLib

  "Standalone vehicle library built from VehicleInterfaces contracts"

  annotation(
    preferredView = "info",
    version = "0.1.0",
    versionDate = "2026-06-21",
    dateModified = "2026-06-21",
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
        Bitmap(extent = {{-74, -76}, {74, 72}}, fileName = "modelica://BobLib/Resources/Images/bobdyn.png")}),
    Documentation(info = "<html>
<p style = \"text-align:center;\">
<img src = \"modelica://BobLib/Resources/Images/bobdyn.png\" width=\"240\"/>
</p>
<p>
BobLib is a standalone vehicle library that starts from the
standard contracts in VehicleInterfaces and builds BobLib components inside
those subsystem boundaries.
</p>
<p>
The authoritative documentation for this package version is
<a href=\"modelica://BobLib.UsersGuide\">BobLib.UsersGuide</a>. BobDocs may
mirror or expand the same material for web reading, but the in-package guide is
the source of truth for current architecture, usage, extension, and validation
guidance.
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
<code>EnergyStorage</code>, <code>ElectricDrives</code>, <code>Engines</code>,
<code>Transmissions</code>, <code>Controllers</code>,
<code>DriverEnvironments</code>, <code>PowerElectronics</code>,
<code>Atmospheres</code>, and <code>Aero</code>.
Regression and component tests are kept in the sibling library
<code>BobLibTest</code> under the repository <code>Tests</code> directory.
Shared mechanics helpers are kept
under <code>Utilities.Mechanics</code>.
</p>
</html>",
      revisions = "<html>
<h4>Version 0.1.0 - 2026-06-21</h4>
<p>
Initial standalone BobLib release aligned with
Modelica Standard Library 4.1.0 and VehicleInterfaces 2.0.2.
</p>
<ul>
<li>Promoted BobLib to the primary standalone package with public subsystem
domains that follow VehicleInterfaces contracts.</li>
<li>Added standard full-vehicle, FMI, and four-post entry points under
<code>Experiments.Standards</code>.</li>
<li>Implemented the publish/subscribe bus architecture for driver intent,
chassis measurements, battery and motor measurements, actuator requests, and
atmosphere-owned ambient signals.</li>
<li>Included the EV battery-inverter-motor-fixed-ratio-transmission-rear-differential
stack, standard VCU, VCU-commanded mechanical brakes, CFD aero map, constant
atmosphere, and detailed double-wishbone suspension/tire/contact models.</li>
<li>Added <code>UsersGuide</code> as the in-library tutorial covering template
execution, vehicle construction, bus usage, lumped model development, core
physics changes, and validation workflow.</li>
<li>Moved production regression coverage to the sibling
<code>Tests/BobLibTest</code> library with pytest-driven translation,
initialization, physics validation, runtime, and regression checks.</li>
</ul>
</html>"));
end BobLib;
