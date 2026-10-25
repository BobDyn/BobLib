within BobLib;
package EnergyStorage

  "Energy-storage models extending VehicleInterfaces energy-storage contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Battery on 2026-10-16; instance-name text removed.
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
          Line(
            points = {{88, 60}, {0, 60}, {0, 10}},
            color = {0, 0, 255}),
          Line(
            points = {{-60, 10}, {-60, 10}, {60, 10}},
            color = {0, 0, 255}),
          Line(
            points = {{-32, -10}, {30, -10}},
            color = {0, 0, 255}),
          Line(
            points = {{0, -10}, {0, -60}, {90, -60}},
            color = {0, 0, 255})}));
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.EnergyStorage</code> contains energy-storage models for the VehicleInterfaces-based vehicle stack.
</p>
<p>
The public battery pack adapter exposes pack voltage, current, and state-of-charge while nested models hold the BobLib battery physics.
</p>
</html>"));
end EnergyStorage;
