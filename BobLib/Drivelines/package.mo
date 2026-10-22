within BobLib;
package Drivelines

  "Driveline models extending VehicleInterfaces driveline contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Driveline on 2026-10-16; instance-name text removed.
  // Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon; (C) 2013-2025, Modelica Association.
  // Modelica License 1.1: Resources/Licenses/LICENSE_VehicleInterfaces.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(
        coordinateSystem(
          preserveAspectRatio = false,
          extent = {{-100, -100}, {100, 100}},
          grid = {1, 1}),
        graphics = {
          Rectangle(
            extent = {{-100, 100}, {100, -100}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.Solid),
          Ellipse(
            extent = {{-64, 50}, {-48, 34}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Sphere,
            fillColor = {215, 215, 215}),
          Rectangle(
            extent = {{-40.7541, 7.9458}, {40.7541, -7.9458}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {215, 215, 215},
            origin = {-21.1593, 21.1992},
            rotation = -31),
          Rectangle(
            extent = {{-91, 10}, {-1, -10}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {215, 215, 215}),

          Ellipse(
            extent = {{-12, 25}, {38, -25}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Sphere,
            fillColor = {215, 215, 215}),
          Ellipse(
            extent = {{6, 8}, {22, -8}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Sphere,
            fillColor = {215, 215, 215}),
          Rectangle(
            extent = {{-40.7541, 7.9458}, {40.7541, -7.9458}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {215, 215, 215},
            origin = {48.8407, -20.8008},
            rotation = -31),
          Ellipse(
            extent = {{76, -34}, {92, -50}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Sphere,
            fillColor = {215, 215, 215})}));

  annotation(Documentation(info = "<html>
<p>
Public driveline domain. BobLib final-drive, differential, halfshaft, and
rear electric PTN assemblies are exposed here through VehicleInterfaces
driveline interfaces.
</p>
</html>"));
end Drivelines;
