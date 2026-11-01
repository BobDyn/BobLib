within BobLib;
package DriverEnvironments

  "Driver-environment models extending VehicleInterfaces driver contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.DriverEnvironment on 2026-10-16; instance-name text removed.
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
            points = {{-82, 54}, {-60, -26}, {-60, -40}, {10, -40}, {18, -26}, {20, -22}, {22,
                -18}, {22, -16}, {22, -14}, {20, -12}, {14, -10}, {8, -10}, {-40, -16}, {-42,
                -12}, {-46, 8}, {-54, 24}, {-62, 34}, {-70, 58}, {-72, 60}, {-76, 62}, {-80,
                62}, {-82, 58}, {-82, 54}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {135, 135, 135}),
          Polygon(
            points = {{24, 36}, {80, 28}, {80, -8}, {40, 8}, {30, 10}, {26, 16}, {26, 30}, {24,
                36}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {170, 170, 255}),
          Line(points = {{80, -8}, {80, -40}, {-80, -40}}),
          Polygon(
            points = {{12, 40}, {6, 6}, {8, 6}, {14, 40}, {12, 40}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {0, 0, 0}),
          Polygon(
            points = {{26, 20}, {10, 24}, {10, 22}, {26, 18}, {26, 20}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {0, 0, 0}),
          Polygon(
            points = {{68, -22}, {62, -30}, {64, -30}, {70, -22}, {68, -22}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {0, 0, 0}),
          Polygon(
            points = {{66, -26}, {72, -26}, {76, -22}, {80, -14}, {80, -18}, {78, -22}, {72, -28},
                {66, -28}, {66, -26}},
            lineColor = {0, 0, 0},
            pattern = LinePattern.None,
            fillPattern = FillPattern.Solid,
            fillColor = {0, 0, 0})}));

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.DriverEnvironments</code> contains
driver-environment models exposed through the standard VehicleInterfaces driver
environment boundary.
</p>
<p>
The minimal driver publisher handles the architecture-neutral commands:
steering wheel, accelerator pedal, and brake pedal. Public adapters may add
architecture-specific pins only when the target vehicle actually needs them, so
EV templates do not inherit manual gear-selection or automatic gearbox-mode
commands by accident.
</p>
</html>"));
end DriverEnvironments;
