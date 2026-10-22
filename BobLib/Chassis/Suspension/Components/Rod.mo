within BobLib.Chassis.Suspension.Components;

model Rod

  import Modelica.Units.SI;

  extends BobLib.Icons.RodIcon;

  parameter BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord pRod(
    m = 0, rCM = zeros(3), inertia = zeros(3, 3))
    "Rod mass properties in the same reference configuration as the pickups" annotation(Dialog(group = "Calibration"));
  parameter Boolean useMass = false "Include rod mass and inertia; disable in equilibrium rigs" annotation(Evaluate = true, choices(checkBox = true), Dialog(group = "Architecture"));

  // Geometry parameters
  parameter SI.Position r_a[3] "Vector from origin to frame_a, expressed in world frame" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter SI.Position r_b[3] "Vector from origin to frame_b, expressed in world frame" annotation(
    Evaluate = false, Dialog(group = "Geometry"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {1, 0, 0} "Axis 1 of universal joint resolved in frame_a (axis 2 is orthogonal to axis 1 and to rod)" annotation(
    Evaluate = false, Dialog(group = "Geometry"));

  parameter Boolean kinematicConstraint = true "Enforce the rod length constraint; disable when the surrounding loop already determines it" annotation(choices(checkBox = true),
    Evaluate = false, Dialog(group = "Geometry"));

  // Visual parameters
  parameter SI.Length linkDiameter "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(
    Evaluate = true, Dialog(tab = "Animation"));
  parameter SI.Length jointDiameter "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(
    Evaluate = true, Dialog(tab = "Animation"));
  parameter Boolean show_universal_axes = true "Draw the universal joint axes in the MultiBody animation" annotation(choices(checkBox = true),
    Evaluate = true, Dialog(tab = "Animation"));

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

protected

  Modelica.Mechanics.MultiBody.Parts.Body body(
    m = pRod.m,
    r_CM = pRod.rCM - r_a,
    I_11 = pRod.inertia[1, 1], I_22 = pRod.inertia[2, 2], I_33 = pRod.inertia[3, 3],
    I_21 = pRod.inertia[2, 1], I_31 = pRod.inertia[3, 1], I_32 = pRod.inertia[3, 2],
    animation = false, useQuaternions = false) if useMass annotation(
    Placement(visible = useMass, transformation(origin = {-8, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Kinematics
  Modelica.Mechanics.MultiBody.Joints.UniversalSpherical rod(n1_a = n1_a,
                                                             rRod_ia = r_b - r_a,
                                                             sphereDiameter = jointDiameter,
                                                             rodWidth = linkDiameter,
                                                             rodHeight = linkDiameter,
                                                             cylinderLength = jointDiameter,
                                                             cylinderDiameter = jointDiameter,
                                                             kinematicConstraint = kinematicConstraint,
                                                             showUniversalAxes = show_universal_axes) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(body.frame_a, rod.frame_ia) annotation(
    Line(visible = useMass, points = {{-8, 50}, {-8, 20}}, color = {95, 95, 95}));
  connect(rod.frame_a, frame_a) annotation(
    Line(points = {{-20, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(rod.frame_b, frame_b) annotation(
    Line(points = {{20, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -40}, {100, 80}}, preserveAspectRatio = true)),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Icon(graphics = {
      Line(origin = {-92, 0}, points = {{-8, 0}, {8, 0}, {8, 0}}),
      Line(origin = {92, 0}, points = {{8, 0}, {-8, 0}, {-8, 0}})
    }),
    Documentation(info = "<html>
<p>
Model <code>Rod</code> represents a two-frame suspension link with optional visual geometry.
</p>
<p>
It transmits multibody forces between hardpoints and is reused by pushrods, pullrods, and steering or linkage elements.
</p>
<p>With <code>useMass=true</code>, an MSL Body attaches to the UniversalSpherical joint's rod-fixed frame. The center of mass is <code>pRod.rCM-r_a</code>; the inertia tensor is about that center, expressed in the reference pickup axes. The universal joint determines rod orientation and removes the otherwise free axial spin of a spherical-spherical rod. Attaching the body therefore adds physical inertia without adding an independent rotational degree of freedom. Dynamic axles supply their tie-rod records; equilibrium rigs remove the body with <code>useMass=false</code>.</p>
</html>"));
end Rod;
