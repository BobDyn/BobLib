within BobLib;
package Brakes

  "Brake implementations for the VehicleInterfaces brakes contract"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Brakes on 2026-10-16; instance-name text removed.
  // Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon; (C) 2013-2025, Modelica Association.
  // Modelica License 1.1: Resources/Licenses/LICENSE_VehicleInterfaces.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(coordinateSystem(preserveAspectRatio = true,  extent = {{-100, -100}, {100, 100}}),
        graphics = {
          Rectangle(
            extent = {{-100, 100}, {100, -100}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.Solid),
          Ellipse(
            extent = {{-80, 80}, {80, -80}},
            lineColor = {95, 95, 95},
            fillColor = {186, 186, 186},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-46, 46}, {46, -46}},
            lineColor = {95, 95, 95},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-28, 28}, {28, -28}},
            lineColor = {95, 95, 95},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-10, 10}, {10, -10}},
            lineColor = {0, 0, 0},
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{22, 30}, {30, 22}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-30, 30}, {-22, 22}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{22, -22}, {30, -30}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-30, -22}, {-22, -30}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),

          Polygon(
            points = {{-88, -20}, {-88, 20}, {-80, 48}, {-48, 44}, {-48, -44}, {-80, -46}, {
                -88, -20}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {215, 215, 215}),
          Line(
            points = {{-66, 10}, {-80, 10}},
            thickness = 0.5),
          Line(
            points = {{-54, -14}, {-60, -10}, {-60, 10}, {-54, 14}},
            thickness = 0.5),
          Line(
            points = {{-64, 30}, {-80, 30}},
            thickness = 0.5),
          Line(
            points = {{-64, -30}, {-80, -30}},
            thickness = 0.5),
          Line(
            points = {{-66, -10}, {-80, -10}},
            thickness = 0.5)}));

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Brakes</code> contains simple
wheel brake implementations that are physically attached at the chassis wheel
hubs while still exposing the standard VehicleInterfaces brakes boundary.
</p>
</html>"));
end Brakes;
