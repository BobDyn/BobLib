within BobLib.Utilities.Mechanics.MultiBody;

model TunableFixedRotation

  "Fixed planar-sequence rotation whose angles stay changeable after compilation"
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Units.Conversions.from_deg;

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(extent = {{84, -16}, {116, 16}})));

  parameter Modelica.Mechanics.MultiBody.Types.RotationSequence sequence = {1, 2, 3} "Axes of the planar rotation sequence" annotation(Evaluate = true);
  parameter Modelica.Units.NonSI.Angle_deg angles[3] = {0, 0, 0} "Rotation angles about the sequence axes";

  // A plain Real matrix keeps the binding live. MSL FixedRotation's final Orientation record is folded to a literal by OMC.
  parameter Real T_rel[3, 3] = Frames.TransformationMatrices.axesRotations(
    sequence,
    from_deg(angles)) "Transformation from frame_a to frame_b";

  final parameter Frames.Orientation R_rel(T = T_rel, w = zeros(3)) "Orientation of frame_b relative to frame_a";

  final parameter Frames.Orientation R_rel_inv(
    T = transpose(T_rel),
    w = zeros(3)) "Orientation of frame_a relative to frame_b";

equation
  Connections.branch(frame_a.R, frame_b.R);
  frame_b.r_0 = frame_a.r_0;

  if Connections.rooted(frame_a.R) then
    frame_b.R = Frames.absoluteRotation(frame_a.R, R_rel);
    zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
    zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t);
  else
    frame_a.R = Frames.absoluteRotation(frame_b.R, R_rel_inv);
    zeros(3) = frame_b.f + Frames.resolve1(R_rel_inv, frame_a.f);
    zeros(3) = frame_b.t + Frames.resolve1(R_rel_inv, frame_a.t);
  end if;
end TunableFixedRotation;
