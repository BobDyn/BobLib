within BobLibTest.TestUtilities.TestMechanics.TestMultiBody.TestContactMechanics;

model TestGroundPhysicsRotatedFrame

  Modelica.Units.SI.Force contactForceWorld[3]
    "Force at the rotated contact frame, resolved in world coordinates";

protected

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity);

  Modelica.Mechanics.MultiBody.Parts.Fixed groundReference(
    animation = false);

  Modelica.Mechanics.MultiBody.Parts.Fixed contactReference(
    r = {0, 0, -0.01},
    animation = false);

  Modelica.Mechanics.MultiBody.Parts.FixedRotation rotatedContactFrame(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,
    n = {1, 1, 0},
    angle = 20,
    animation = false);

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground(
    c = 100000,
    d = 0,
    eps = 1e-6,
    forceEps = 1);

equation
  contactForceWorld = Modelica.Mechanics.MultiBody.Frames.resolve1(
    ground.frame_b.R,
    ground.frame_b.f);

  connect(groundReference.frame_b, ground.frame_a);
  connect(contactReference.frame_b, rotatedContactFrame.frame_a);
  connect(rotatedContactFrame.frame_b, ground.frame_b);

  annotation(
    experiment(StartTime = 0, StopTime = 0.01));
end TestGroundPhysicsRotatedFrame;
