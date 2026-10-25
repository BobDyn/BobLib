within BobLib.Chassis.Suspension.Components;

model ForceOnlyRod

  import Modelica.Units.SI;
  import Modelica.Math.Vectors.norm;

  extends BobLib.Icons.RodIcon;

  // Geometry parameters
  parameter SI.Position r_a[3] "Vector from origin to frame_a, expressed in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position r_b[3] "Vector from origin to frame_b, expressed in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Force EA = 8.6e6 "Elastic modulus multiplied by cross-sectional area" annotation(Dialog(group = "Operating settings"));
  parameter SI.TranslationalDampingConstant d = 2e3 "Axial damping" annotation(Dialog(group = "Operating settings"));

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

  replaceable Modelica.Mechanics.Translational.Components.SpringDamper springDamper(
    s_rel0 = norm(r_b - r_a),
    c = EA/norm(r_b - r_a),
    d = d,
    s_rel(
      start = norm(r_b - r_a),
      fixed = true)) constrainedby Modelica.Mechanics.Translational.Interfaces.PartialCompliant annotation(choicesAllMatching = true,
    Placement(transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(frame_a, lineForceWithMass.frame_a) annotation(
    Line(points = {{-100, 0}, {-20, 0}}));
  connect(lineForceWithMass.frame_b, frame_b) annotation(
    Line(points = {{20, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(springDamper.flange_a, lineForceWithMass.flange_a) annotation(
    Line(points = {{-10, 40}, {-12, 40}, {-12, 20}}, color = {0, 127, 0}));
  connect(springDamper.flange_b, lineForceWithMass.flange_b) annotation(
    Line(points = {{10, 40}, {12, 40}, {12, 20}}, color = {0, 127, 0}));
  annotation(Diagram,
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Icon(graphics = {
      Line(origin = {-92, 0}, points = {{-8, 0}, {8, 0}, {8, 0}}),
      Line(origin = {92, 0}, points = {{8, 0}, {-8, 0}, {-8, 0}})
    }),
    Documentation(info = "<html>
<p>
Model <code>ForceOnlyRod</code> represents a massless axial spring-damper link.
The hardpoint separation defines its free length, and <code>EA</code> divided by that length defines its axial stiffness.
It does not impose a rigid length constraint or introduce universal-joint axes.
</p>
<p>
It is used where only the axial load path is needed and additional inertia or visual geometry would add unnecessary model detail.
</p>
</html>"));
end ForceOnlyRod;