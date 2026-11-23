within BobLibTest.Utilities.Mechanics.MultiBody;

model TestTunableFixedRotation

  "TunableFixedRotation must place frame_b exactly where MSL FixedRotation does"

  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false);
  parameter Modelica.Units.NonSI.Angle_deg angles[3] = {1.5, 0, 0.5};

  BobLib.Utilities.Mechanics.MultiBody.TunableFixedRotation tunable(sequence = {1, 2, 3}, angles = angles);

  Modelica.Mechanics.MultiBody.Parts.FixedRotation reference(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence = {1, 2, 3},
    angles = angles,
    animation = false);

  Modelica.Mechanics.MultiBody.Parts.Body tunableBody(m = 1, r_CM = {0.1, 0.2, 0.3}, animation = false);

  Modelica.Mechanics.MultiBody.Parts.Body referenceBody(m = 1, r_CM = {0.1, 0.2, 0.3}, animation = false);

equation
  connect(world.frame_b, tunable.frame_a);
  connect(world.frame_b, reference.frame_a);
  connect(tunable.frame_b, tunableBody.frame_a);
  connect(reference.frame_b, referenceBody.frame_a);
  assert(
    max(abs(tunable.frame_b.R.T - reference.frame_b.R.T)) < 1e-12,
    "TunableFixedRotation orientation differs from FixedRotation");
  assert(
    max(abs(tunable.frame_a.t - reference.frame_a.t)) < 1e-9,
    "TunableFixedRotation reaction torque differs from FixedRotation");
  annotation(Documentation(info = "<html><p>Coverage fixture for <code>BobLib.Utilities.Mechanics.MultiBody.TunableFixedRotation</code>.</p></html>"));
end TestTunableFixedRotation;
