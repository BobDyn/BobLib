within BobLib;
package Chassis

  "Chassis-domain models built from VehicleInterfaces chassis contracts"

  // Package icon adapted from VehicleInterfaces 2.0.2 Icons.Chassis on 2026-10-16; instance-name text removed.
  // Copyright (C) 2005-2013, Dassault Systemes, DLR and Modelon; (C) 2013-2025, Modelica Association.
  // Modelica License 1.1: Resources/Licenses/LICENSE_VehicleInterfaces.txt.
  extends Modelica.Icons.Package;

  annotation(Icon(coordinateSystem(
          preserveAspectRatio = false,
          extent = {{-160, -100}, {160, 100}},
          grid = {1, 1}), graphics = {
          Rectangle(
            extent = {{-160, 100}, {160, -100}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 255},
            fillPattern = FillPattern.Solid, lineThickness = 0.5, pattern = LinePattern.Solid),
          Ellipse(
            extent = {{-75, -10}, {-135, -70}},
            lineColor = {0, 0, 0},
            fillColor = {0, 0, 0},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-85, -20}, {-125, -60}},
            lineColor = {0, 0, 0},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{-109, -36}, {-101, -44}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Line(
            points = {{-105, -22}, {-105, -58}, {-105, -40}, {-90, -31}, {-105, -40}, {-90,
                -49}, {-120, -31}, {-105, -40}, {-120, -48}},
            color = {95, 95, 95},
            thickness = 1),
          Ellipse(
            extent = {{-100, -35}, {-110, -45}},
            lineColor = {95, 95, 95},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{120, -10}, {60, -70}},
            lineColor = {0, 0, 0},
            fillColor = {0, 0, 0},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{110, -20}, {70, -60}},
            lineColor = {0, 0, 0},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Ellipse(
            extent = {{86, -36}, {94, -44}},
            lineColor = {0, 0, 255},
            pattern = LinePattern.None,
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Line(
            points = {{90, -22}, {90, -58}, {90, -40}, {105, -31}, {90, -40}, {105, -49}, {75,
                -31}, {90, -40}, {75, -48}},
            color = {95, 95, 95},
            thickness = 1),
          Ellipse(
            extent = {{95, -35}, {85, -45}},
            lineColor = {95, 95, 95},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-142, -36}, {-149, -34}, {-154, -22}, {-154, -9}, {-151, 2}, {-142,
                12}, {-130, 19}, {-103, 24}, {-84, 27}, {-74, 29}, {-67, 31}, {-61, 33}, {-23,
                62}, {-11, 68}, {2, 70}, {97, 70}, {115, 64}, {147, 27}, {150, 23}, {151, 18},
                {151, 0}, {152, -4}, {154, -7}, {154, -16}, {139, -35}, {128, -35}, {124, -23},
                {122, -19}, {118, -15}, {113, -10}, {103, -5}, {96, -3}, {88, -3}, {82, -4},
                {77, -5}, {69, -10}, {63, -15}, {59, -20}, {54, -28}, {50, -35}, {46, -38}, {
                41, -39}, {-62, -39}, {-67, -35}, {-69, -28}, {-72, -22}, {-77, -14}, {-86,
                -8}, {-92, -5}, {-101, -3}, {-109, -3}, {-118, -5}, {-128, -11}, {-134, -17},
                {-139, -26}, {-142, -36}},
            lineColor = {95, 95, 95},
            fillColor = {255, 255, 170},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{25, 62}, {11, 26}, {-39, 23}, {-41, 36}, {-20, 54}, {-6, 61}, {25, 62}},
            lineColor = {0, 0, 0},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{73, 62}, {39, 62}, {31, 27}, {83, 30}, {73, 62}},
            lineColor = {0, 0, 0},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{122, 30}, {105, 30}, {80, 62}, {91, 62}, {110, 54}, {120, 38}, {122, 30}},
            lineColor = {0, 0, 0},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Line(points = {{77, 62}, {93, 25}, {89, 8}, {78, -2}}, color = {95, 95, 95}),
          Line(points = {{32, 62}, {19, 18}, {19, -15}, {25, -33}}, color = {95, 95, 95}),
          Line(points = {{144, 22}, {87, 22}, {17, 18}, {-86, 9}}, color = {95, 95, 95}),
          Line(points = {{-51, 23}, {-54, 12}, {-52, -14}, {-48, -32}}, color = {95, 95, 95}),
          Line(points = {{61, -16}, {-68, -15}}, color = {95, 95, 95}),
          Polygon(points = {{109, 25}, {98, 25}, {98, 16}, {109, 16}, {109, 25}},
              lineColor = {95, 95, 95}),
          Line(points = {{154, -15}, {145, -15}, {120, -16}, {151, -3}}, color = {95, 95, 95}),
          Polygon(
            points = {{-67, -35}, {-62, -39}, {41, -39}, {46, -38}, {50, -35}, {52, -33}, {-68,
                -33}, {-67, -35}},
            lineColor = {0, 0, 255},
            fillColor = {255, 255, 85},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{128, -35}, {139, -35}, {155, -15}, {119, -16}, {123, -21}, {128, -35}},
            lineColor = {0, 0, 255},
            fillColor = {255, 255, 85},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-142, -35}, {-139, -26}, {-136, -20}, {-154, -20}, {-154, -22}, {-149,
                -34}, {-142, -36}, {-142, -35}},
            lineColor = {0, 0, 0},
            fillColor = {255, 255, 85},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-154, -9}, {-151, 2}, {-146, 8}, {-145, 2}, {-147, -7}, {-154, -9}},
            lineColor = {95, 95, 95},
            fillColor = {215, 215, 215},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{151, -1}, {146, 0}, {143, 12}, {151, 17}, {151, -1}},
            lineColor = {95, 95, 95},
            fillColor = {255, 0, 0},
            fillPattern = FillPattern.Solid),
          Polygon(
            points = {{-41, 36}, {-47, 30}, {-48, 25}, {-46, 23}, {-39, 23}, {-41, 36}},
            lineColor = {0, 0, 0},
            fillColor = {95, 95, 95},
            fillPattern = FillPattern.Solid)}));

  annotation(Documentation(info = "<html>
<p>
Public chassis domain for BobLib. Body, suspension, and
complete chassis models live here because they are part of the VehicleInterfaces
chassis subsystem boundary.
</p>
<p>
Direct chassis models are VehicleInterfaces-facing chassis contract adapters:
partial adapters define the shared boundary, and concrete variants bind records
and detailed physics for vehicle experiments. Detailed implementation cores live
in <code>Internal</code>; body physics lives in <code>Body</code>; simple
wheel-speed-opposing brake models live in <code>Brakes</code>; suspension, axle,
tire, and raw contact-patch frame models live in <code>Suspension</code> and
its nested packages. Shared MultiBody contact mechanics live under
<code>Utilities.Mechanics.MultiBody</code> and are wired at the chassis or rig
assembly level.
</p>
</html>"));
end Chassis;
