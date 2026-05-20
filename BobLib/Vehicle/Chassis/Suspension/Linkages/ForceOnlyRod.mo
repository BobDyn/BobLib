within BobLib.Vehicle.Chassis.Suspension.Linkages;

model ForceOnlyRod
  import Modelica.SIunits;
  import Modelica.Math.Vectors.norm;
  // Geometry parameters
  parameter SIunits.Position r_a[3] "Vector from origin to frame_a, expressed in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Position r_b[3] "Vector from origin to frame_b, expressed in world frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SIunits.Force EA = 8.6e6 "Elastic modulus multiplied by cross-sectional area";
  parameter SIunits.TranslationalDampingConstant d = 2e3 "Axial damping";
  parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {1, 0, 0} "Axis 1 of universal joint resolved in frame_a (axis 2 is orthogonal to axis 1 and to rod)" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter Boolean kinematicConstraint = false annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  // Visual parameters
  parameter SIunits.Length linkDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  parameter SIunits.Length jointDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  parameter Boolean show_universal_axes = true annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper(s_rel0 = norm(r_b - r_a), c = EA/norm(r_b - r_a), d = d, s_rel(start = norm(r_b - r_a), fixed = true)) annotation(
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
  annotation(
    Diagram(graphics),
    Icon(graphics = {Line(origin = {-25.8, 3.2}, points = {{-54.2, -3.2}, {25.8, -3.2}, {105.8, -3.2}}, thickness = 5), Ellipse(origin = {-80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {80, 0}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-92, 0}, points = {{-8, 0}, {8, 0}, {8, 0}}), Line(origin = {92, 0}, points = {{8, 0}, {-8, 0}, {-8, 0}})}),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end ForceOnlyRod;
