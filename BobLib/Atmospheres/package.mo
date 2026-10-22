within BobLib;
package Atmospheres

  "Atmosphere models extending VehicleInterfaces atmosphere contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Atmosphere on 2026-10-16; instance-name text removed.
  // Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon; (C) 2013-2025, Modelica Association.
  // Modelica License 1.1: Resources/Licenses/LICENSE_VehicleInterfaces.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(
        coordinateSystem(
          preserveAspectRatio = false,
          extent = {{-200, -100}, {200, 100}},
          grid = {2, 2}),
        graphics = {
          Rectangle(
            extent = {{-200, 100}, {200, -100}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.Solid,
            fillColor = {201, 210, 255},
            fillPattern = FillPattern.Solid, lineThickness = 0.5),
          Ellipse(
            extent = {{-166, 29}, {-132, 5}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-158, 47}, {-66, 5}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-150, 57}, {-116, 33}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-142, 19}, {-108, -5}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-118, 27}, {-74, 3}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-86, 37}, {-52, 13}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-124, 53}, {-70, 29}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{118, 62}, {177, 2}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 0},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{109, 32}, {67, 27}, {69, 18}, {108, 25}, {109, 26}, {109, 32}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 0},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{116, 11}, {89, -14}, {95, -19}, {120, 5}, {116, 11}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 0},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{138, -6}, {130, -38}, {137, -40}, {144, -8}, {138, -6}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {255, 255, 0},
            fillPattern = FillPattern.Solid)}));

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Atmospheres</code> contains atmosphere
models exposed through the standard VehicleInterfaces atmosphere contract.
</p>
<p>
Atmosphere models implement the VehicleInterfaces replaceable functions.
Aero queries local density and wind at its load point through the shared
<code>inner atmosphere</code>. No additional connector contract is required.
</p>
</html>"));
end Atmospheres;
