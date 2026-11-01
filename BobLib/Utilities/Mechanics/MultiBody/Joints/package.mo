within BobLib.Utilities.Mechanics.MultiBody;

package Joints

  "Joint assemblies with local Modelica compatibility fixes"

  // Package graphic adapted from MSL 4.1.0 Mechanics/MultiBody/Joints/package.mo on 2026-10-16.
  // Copyright (c) 1998-2025, Modelica Association and contributors.
  // BSD-3-Clause: Resources/Licenses/LICENSE_ModelicaStandardLibrary.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(
      coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}),
      graphics = {
        Rectangle(
          origin = {30, -22},
          lineColor = {64, 64, 64},
          fillColor = {235, 235, 235},
          fillPattern = FillPattern.VerticalCylinder,
          extent = {{-20, 0}, {20, 100}},
          rotation = 110),
        Rectangle(
          origin = {30, -22},
          lineColor = {64, 64, 64},
          fillColor = {215, 215, 215},
          fillPattern = FillPattern.VerticalCylinder,
          extent = {{-20, 0}, {20, 100}}),
        Rectangle(
          origin = {30, -22},
          lineColor = {64, 64, 64},
          extent = {{-20, 0}, {20, 100}}),
        Rectangle(
          origin = {30, -22},
          lineColor = {64, 64, 64},
          extent = {{-20, 0}, {20, 100}},
          rotation = 110),
        Ellipse(
          origin = {30, -22},
          rotation = -45,
          lineColor = {64, 64, 64},
          fillColor = {255, 255, 255},
          fillPattern = FillPattern.HorizontalCylinder,
          extent = {{-34, -34}, {34, 34}}),
        Ellipse(
          origin = {30, -22},
          lineColor = {64, 64, 64},
          fillColor = {64, 64, 64},
          fillPattern = FillPattern.Solid,
          extent = {{-14, 14}, {14, -14}}),
        Ellipse(
          extent = {{-34, 34}, {34, -34}},
          lineColor = {64, 64, 64},
          origin = {30, -22}), Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}, lineThickness = 0.5, fillPattern = FillPattern.None)}));

end Joints;
