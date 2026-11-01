within BobLib.Utilities.Mechanics.MultiBody.ContactMechanics;
model RoadGround "Unilateral point contact with a shared VI road"

  extends BobLib.Icons.GroundPhysicsIcon;
  parameter Modelica.Units.SI.Position r[3] = zeros(3) "Surface coordinate start guesses and calibrated normal-height datum" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.TranslationalSpringConstant c(min = 0) = 100000 "Normal contact stiffness relating smoothed penetration to compressive force" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.TranslationalDampingConstant d(min = 0) = 750 "Normal contact damping acting on the rate of smoothed penetration" annotation(Dialog(group = "Operating settings"));
  parameter Boolean flatRoadFastPath = false "Use the horizontal z=0 road specialization; requires a compatible VI road" annotation(Dialog(group = "Advanced"), choices(checkBox = true), Evaluate = true);
  parameter Modelica.Units.SI.Length eps(min = Modelica.Constants.small) = 1e-6 "Length scale smoothing the transition between separation and penetration" annotation(Dialog(group = "Advanced"));
  parameter Modelica.Units.SI.Force forceEps(min = Modelica.Constants.small) = 1 "Force scale smoothing the cutoff of tensile contact force" annotation(Dialog(group = "Advanced"));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));

  RoadSurface surface(
    final flatRoadFastPath = flatRoadFastPath,
    r = frame_b.r_0,
    s_start = r[1],
    w_start = r[2]) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-20, -12}, {20, 12}})));
  output Modelica.Units.SI.Length pen;
  output Modelica.Units.SI.Force f_z "Compressive normal force";

protected
  Modelica.Units.SI.Force f_raw;
  Modelica.Units.SI.Position gap;

equation
  assert(
    c>=0 and d>=0 and eps>0 and forceEps>0,
    "RoadGround requires nonnegative stiffness/damping and positive smoothing scales");
  gap = surface.gap-r[3];
  pen = smooth(1, 0.5*(sqrt(gap^2+eps^2)-gap));
  f_raw = c*pen+d*der(pen);
  f_z = smooth(1, 0.5*(sqrt(f_raw^2+forceEps^2)+f_raw));
  frame_b.f = Modelica.Mechanics.MultiBody.Frames.resolve2(frame_b.R, -f_z*surface.normal);
  frame_b.t = zeros(3);
  annotation(Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})), Icon(graphics = {
    Line(points = {{-60, -45}, {60, -45}}, thickness = 1),
    Line(points = {{-60, -45}, {-40, -75}}), Line(points = {{-30, -45}, {-10, -75}}),
    Line(points = {{0, -45}, {20, -75}}), Line(points = {{30, -45}, {50, -75}})}),
    Documentation(info = "<html><p>Uses the same smooth unilateral contact law as FixedGround, resolved along the VI road normal. The road is an immovable support. r[3] offsets the normal contact datum to preserve the nominal tire/hardpoint setup; r[1:2] are surface-coordinate start guesses. Raw contact-patch geometry remains owned by the tire wheel model; this component does not introduce a circular-wheel contact approximation.</p></html>"));
end RoadGround;
