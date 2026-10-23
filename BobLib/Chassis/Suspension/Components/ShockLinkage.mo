within BobLib.Chassis.Suspension.Components;

model ShockLinkage

  import Modelica.Units.SI;

  extends BobLib.Icons.ShockLinkageIcon;

  // Spring parameters
  parameter SI.Length s_0 "Spring free length" annotation(
    Dialog(group = "Spring Params"));
  parameter Integer springPoints(min = 2, max = size(springTable, 1)) = size(springTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Spring Params"));
  parameter Real springTable[:, 2] "Table of spring force vs deflection (change in length)" annotation(
    Dialog(group = "Spring Params"));

  // Damper parameters
  parameter Integer damperPoints(min = 2, max = size(damperTable, 1)) = size(damperTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Damper Params"));
  parameter Real damperTable[:, 2] "Table of damper force vs relative velocity" annotation(
    Dialog(group = "Damper Params"));

  // Advanced parameters
  parameter Boolean fixedRotationAtFrame_a = false "Fix the joint orientation at frame_a when the linkage requires a rotational reference" annotation(choices(checkBox = true), Dialog(tab = "Advanced"));
  parameter Boolean fixedRotationAtFrame_b = false "Fix the joint orientation at frame_b when the linkage requires a rotational reference" annotation(choices(checkBox = true), Dialog(tab = "Advanced"));

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

  parameter Boolean useDamping = true "Include the parallel damper; rigs disable it for static equilibrium" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));

  // Force elements
  replaceable BobLib.Chassis.Suspension.Components.TabularSpring TabularSpring(
    springPoints = springPoints, springTable = springTable,
    s_0 = s_0) constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges annotation(choicesAllMatching = true,
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));

  replaceable BobLib.Chassis.Suspension.Components.TabularDamper TabularDamper(
    damperPoints = damperPoints,
    damperTable = damperTable) if useDamping constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges annotation(choicesAllMatching = true,
    Placement(visible = useDamping, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}})));

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
    Line(visible = useDamping, points = {{-10, 50}, {-20, 50}, {-20, 10}, {-6, 10}}, color = {0, 127, 0}));
  connect(TabularDamper.flange_b, lineForceWithMass.flange_b) annotation(
    Line(visible = useDamping, points = {{10, 50}, {20, 50}, {20, 10}, {6, 10}}, color = {0, 127, 0}));
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
