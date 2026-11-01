within BobLib;
package Engines

  "Engine models extending VehicleInterfaces engine contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Engine on 2026-10-16; instance-name text removed.
  // Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon; (C) 2013-2025, Modelica Association.
  // Modelica License 1.1: Resources/Licenses/LICENSE_VehicleInterfaces.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}),
        graphics = {
          Rectangle(
            extent = {{-100, 100}, {100, -100}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.Solid),
          Polygon(
            points = {{-56, 62}, {36, 76}, {46, 52}, {-46, 38}, {-56, 62}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{58, 8}, {-34, -6}, {-34, -46}, {58, -32}, {58, 8}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillColor = {135, 135, 135},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-58, -38}, {-34, -46}, {-34, -6}, {-16, 28}, {-36, 54}, {-46, 38}, {-56,
                62}, {-76, 48}, {-58, 2}, {-58, -38}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {175, 175, 175},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-16, 28}, {76, 42}, {58, 8}, {-34, -6}, {-16, 28}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-36, 54}, {-16, 28}, {76, 42}, {56, 68}, {-36, 54}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-18, 54}, {2, 34}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{40, 62}, {60, 42}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{12, 58}, {32, 38}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Line(points = {{-46, 38}, {-36, 54}, {40, 66}})}));

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Engines</code> contains engine subsystem
models exposed through the standard VehicleInterfaces engine boundary.
</p>
<p>
Direct models in this package are intended as vehicle-level insertion points.
Detailed internal-combustion engine physics should live one package level
deeper, then be adapted here through
<code>BobLib.Engines.Interfaces.Base</code>.
</p>
<p>
<code>SimpleICEngine</code> is a small sample implementation for architecture
work. Inactive or omitted engine behavior should be represented by choosing an
architecture without an engine subsystem rather than by carrying a no-physics
adapter in the public package surface.
</p>
</html>"));
end Engines;
