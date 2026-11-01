within BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics;

model TestFixedGroundRotatedFrame

  Modelica.Units.SI.Force contactForceWorld[3]
    "Force at the rotated contact frame, resolved in world coordinates";

protected

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(transformation(origin = {-150, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Fixed contactReference(
    r = {0, 0, -0.01},
    animation = false) annotation(Placement(transformation(origin = {-110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation rotatedContactFrame(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,
    n = {1, 1, 0},
    angle = 20,
    animation = false) annotation(Placement(transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.FixedGround ground(
    r = {0, 0, 0.02},
    c = 100000,
    d = 0,
    eps = 1e-6,
    forceEps = 1) annotation(Placement(transformation(origin = {60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

equation
  contactForceWorld = Modelica.Mechanics.MultiBody.Frames.resolve1(
    ground.frame_b.R,
    ground.frame_b.f);

  assert(abs(contactForceWorld[3] + 3000) < 0.01, "Fixed ground contact force or reference height changed");
  assert(abs(contactForceWorld[1]) + abs(contactForceWorld[2]) < 1e-6, "Contact force is not world-vertical");
  connect(contactReference.frame_b, rotatedContactFrame.frame_a) annotation(Line(points = {{-100, -20}, {-40, -20}}, color = {95, 95, 95}));
  connect(rotatedContactFrame.frame_b, ground.frame_b) annotation(Line(points = {{-20, -20}, {-17, -20}, {-17, 6}, {60, 6}, {60, 0}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-160, -100}, {120, 100}}, preserveAspectRatio = true)));
end TestFixedGroundRotatedFrame;
