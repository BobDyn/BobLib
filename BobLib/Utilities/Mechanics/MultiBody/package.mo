within BobLib.Utilities.Mechanics;
package MultiBody

  "Reusable MultiBody helpers shared across vehicle domains"

  // Package graphic adapted from MSL 4.1.0 Mechanics/MultiBody/package.mo on 2026-10-16.
  // Copyright (c) 1998-2025, Modelica Association and contributors.
  // BSD-3-Clause: Resources/Licenses/LICENSE_ModelicaStandardLibrary.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100},
            {100, 100}}), graphics = {
        Polygon(
          points = {{-58, 76}, {6, 76}, {-26, 50}, {-58, 76}},
          lineColor = {95, 95, 95},
          fillColor = {255, 255, 255},
          fillPattern = FillPattern.Solid),
        Line(
          points = {{-26, 50}, {28, -50}}),
        Ellipse(
          extent = {{-4, -14}, {60, -78}},
          lineColor = {135, 135, 135},
          fillPattern = FillPattern.Sphere,
          fillColor = {255, 255, 255}), Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities.Mechanics.MultiBody</code>
contains reusable MultiBody helpers that are not owned by one vehicle subsystem.
</p>
<p>
Use this package for frame, fixture, actuator, and contact utilities that are
consumed by chassis-level assemblies or by more than one physics domain. Domain
models should expose raw frames and use these utilities at the assembly level
rather than burying shared MultiBody behavior inside tire, suspension, or
powertrain subpackages.
</p>
</html>"));
end MultiBody;
