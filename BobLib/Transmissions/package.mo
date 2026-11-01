within BobLib;
package Transmissions

  "Transmission models extending VehicleInterfaces transmission contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Transmission on 2026-10-16; instance-name text removed.
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
          Rectangle(
            extent = {{-90, 10}, {-60, -10}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {192, 192, 192}),
          Rectangle(
            extent = {{90, 10}, {60, -10}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {192, 192, 192}),
          Polygon(
            points = {{-60, -60}, {-60, 60}, {-50, 60}, {-30, 40}, {30, 40}, {40, 10}, {60, 10},
                {60, -10}, {40, -10}, {30, -40}, {20, -40}, {15, -35}, {6, -45}, {-30, -45},
                {-40, -38}, {-49, -60}, {-60, -60}},
            lineColor = {0, 0, 0},
            fillColor = {192, 192, 192},
            fillPattern = FillPattern.Solid),
          Line(points = {{-53, 0}, {-36, 0}}),
          Line(points = {{-53, 23}, {-45, 23}, {-38, 18}}),
          Line(points = {{-53, 36}, {-44, 36}, {-36, 30}}),
          Line(points = {{-55, -19}, {-47, -19}, {-40, -16}}),
          Line(points = {{-53, -35}, {-46, -35}, {-43, -31}}),
          Line(points = {{-27, 29}, {21, 29}, {28, 14}}),
          Line(points = {{-9, -1}, {21, -1}}),
          Line(points = {{-33, -34}, {-30, -34}, {7, -34}})}));

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Transmissions</code> contains
transmission subsystem models exposed through the standard VehicleInterfaces
transmission boundary.
</p>
<p>
Direct models in this package are vehicle-level insertion points. Reusable
gear, shift, clutch, launch, and loss physics should live one package level
deeper, then be adapted here through
<code>BobLib.Transmissions.Interfaces.Base</code> or one of its
specialized transmission interfaces.
</p>
</html>"));
end Transmissions;
