within BobLibVehicleInterfaces.Chassis.Suspension.Linkages;

model ShockLinkage

  extends BobLibVehicleInterfaces.Icons.ShockLinkageIcon;

  import SI = Modelica.Units.SI;

// Geometry parameters
  parameter SI.Position r_a[3] "Initial vector from origin to frame_a, resolved in world frame" annotation(
    Dialog(group = "Geometry"));
  parameter SI.Position r_b[3] "Initial vector from origin to frame_b, resolved in world frame" annotation(
    Dialog(group = "Geometry"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n_a "Axis of revolute joint 1, resolved in world frame" annotation(
    Dialog(group = "Geometry"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n_b "Axis of revolute joint 2, resolved in world frame" annotation(
    Dialog(group = "Geometry"));

  // Spring parameters
  parameter SI.Length s_0 "Spring free length" annotation(
    Dialog(group = "Spring Params"));
  parameter SI.TranslationalSpringConstant springTable[:, 2] "Table of spring force vs deflection (change in length)" annotation(
    Dialog(group = "Spring Params"));

  // Damper parameters
  parameter SI.TranslationalDampingConstant damperTable[:, 2] "Table of damper force vs relative velocity" annotation(
    Dialog(group = "Damper Params"));

  // Visual parameters (implement visuals later)
  parameter SI.Length linkDiameter "Link diameter" annotation(
    Dialog(tab = "Animation"));
  parameter SI.Length jointDiameter "Joint diameter" annotation(
    Dialog(tab = "Animation"));

  // Advanced parameters
  parameter Boolean fixedRotationAtFrame_a = false annotation(tab = "Advanced");
  parameter Boolean fixedRotationAtFrame_b = false annotation(tab = "Advanced");

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

  // Force elements
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.TabularSpring TabularSpring(
    springTable = springTable,
    s_0 = s_0) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.TabularDamper TabularDamper(damperTable = damperTable) annotation(
    Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));

  // Shock axis
  Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass(
    fixedRotationAtFrame_a = fixedRotationAtFrame_a,
    fixedRotationAtFrame_b = fixedRotationAtFrame_b) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  connect(frame_a, lineForceWithMass.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(lineForceWithMass.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(TabularSpring.flange_a, lineForceWithMass.flange_a) annotation(
    Line(points = {{-10, 30}, {-20, 30}, {-20, 10}, {-6, 10}}, color = {0, 127, 0}));
  connect(TabularSpring.flange_b, lineForceWithMass.flange_b) annotation(
    Line(points = {{10, 30}, {20, 30}, {20, 10}, {6, 10}}, color = {0, 127, 0}));
  connect(TabularDamper.flange_a, lineForceWithMass.flange_a) annotation(
    Line(points = {{-10, 50}, {-20, 50}, {-20, 10}, {-6, 10}}, color = {0, 127, 0}));
  connect(TabularDamper.flange_b, lineForceWithMass.flange_b) annotation(
    Line(points = {{10, 50}, {20, 50}, {20, 10}, {6, 10}}, color = {0, 127, 0}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram,
    Icon(graphics = {
      Line(origin = {-66, 0}, points = {{-14, 0}, {6, 0}}, thickness = 3),
      Line(origin = {67, 0}, points = {{-7, 0}, {9, 0}}, thickness = 3)
    }),
    Documentation(info = "<html>
<p>
Model <code>ShockLinkage</code> connects a spring and damper between two suspension hardpoints.
</p>
<p>
It uses tabular spring and damper characteristics so vehicle records can supply measured or tuned rate curves.
</p>
</html>"));
end ShockLinkage;