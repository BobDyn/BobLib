within BobLibTest.Utilities.Mechanics.MultiBody;

model TestTunableFixedRotation

  "TunableFixedRotation must place frame_b exactly where MSL FixedRotation does"

  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  parameter Modelica.Units.NonSI.Angle_deg angles[3] = {1.5, 0, 0.5} "Shared camber, zero and toe angles for both rotations";

  BobLib.Utilities.Mechanics.MultiBody.TunableFixedRotation tunable(sequence = {1, 2, 3}, angles = angles) annotation(
    Placement(transformation(origin = {-20, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation reference(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence = {1, 2, 3},
    angles = angles,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body tunableBody(m = 1, r_CM = {0.1, 0.2, 0.3}, animation = false) annotation(
    Placement(transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body referenceBody(m = 1, r_CM = {0.1, 0.2, 0.3}, animation = false) annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(world.frame_b, tunable.frame_a) annotation(
    Line(points = {{-80, -90}, {-50, -90}, {-50, 30}, {-30, 30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(world.frame_b, reference.frame_a) annotation(
    Line(points = {{-80, -90}, {-50, -90}, {-50, -30}, {-30, -30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(tunable.frame_b, tunableBody.frame_a) annotation(
    Line(points = {{-10, 30}, {20, 30}}, color = {95, 95, 95}, thickness = 0.5));
  connect(reference.frame_b, referenceBody.frame_a) annotation(
    Line(points = {{-10, -30}, {20, -30}}, color = {95, 95, 95}, thickness = 0.5));
  assert(
    max(abs(tunable.frame_b.R.T - reference.frame_b.R.T)) < 1e-12,
    "TunableFixedRotation orientation differs from FixedRotation");
  assert(
    max(abs(tunable.frame_a.t - reference.frame_a.t)) < 1e-9,
    "TunableFixedRotation reaction torque differs from FixedRotation");
  annotation(
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
    Documentation(info = "<html><p>Coverage fixture for <code>BobLib.Utilities.Mechanics.MultiBody.TunableFixedRotation</code>.</p></html>"));
end TestTunableFixedRotation;
