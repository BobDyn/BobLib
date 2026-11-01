within BobLib;
package ElectricDrives

  "Electric-drive models extending VehicleInterfaces electric-drive contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.ElectricMotor on 2026-10-16; instance-name text removed.
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
          Rectangle(
            extent = {{-50, 70}, {70, -30}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {0, 128, 255}),
          Rectangle(
            extent = {{-50, 60}, {-70, -30}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {128, 128, 128}),
          Rectangle(
            extent = {{70, 10}, {90, -10}},
            lineColor = {0, 0, 0},
            fillPattern = FillPattern.HorizontalCylinder,
            fillColor = {95, 95, 95}),
          Rectangle(
            extent = {{-50, 80}, {30, 60}},
            lineColor = {95, 95, 95},
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-60, -50}, {-50, -50}, {-30, -10}, {40, -10}, {60, -50}, {70, -50}, {
                70, -60}, {-60, -60}, {-60, -50}},
            lineColor = {0, 0, 0},
            fillColor = {0, 0, 0},
            fillPattern = FillPattern.Solid)}));
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.ElectricDrives</code> contains electric-drive models for the VehicleInterfaces-based vehicle stack.
</p>
<p>
The public motor adapter is the first-level insertion point; nested models contain BobLib-specific torque and power limiting behavior.
</p>
</html>"));
end ElectricDrives;
