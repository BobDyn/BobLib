within BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics;

model TestGroundPhysicsRotatedFrame

  Modelica.Units.SI.Force contactForceWorld[3]
    "Force at the rotated contact frame, resolved in world coordinates";

protected

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(transformation(origin = {-170, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Fixed groundReference(
    animation = false) annotation(Placement(transformation(origin = {-60, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Fixed contactReference(
    r = {0, 0, -0.01},
    animation = false) annotation(Placement(transformation(origin = {-140, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation rotatedContactFrame(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis,
    n = {1, 1, 0},
    angle = 20,
    animation = false) annotation(Placement(transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics ground(
    c = 100000,
    d = 0,
    eps = 1e-6,
    forceEps = 1) annotation(Placement(transformation(origin = {60, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

equation
  contactForceWorld = Modelica.Mechanics.MultiBody.Frames.resolve1(
    ground.frame_b.R,
    ground.frame_b.f);

  connect(groundReference.frame_b, ground.frame_a) annotation(Line(points = {{-50, -70}, {60, -70}, {60, -50}}, color = {95, 95, 95}));
  connect(contactReference.frame_b, rotatedContactFrame.frame_a) annotation(Line(points = {{-130, -30}, {-70, -30}}, color = {95, 95, 95}));
  connect(rotatedContactFrame.frame_b, ground.frame_b) annotation(Line(points = {{-50, -30}, {40, -30}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-180, -120}, {120, 120}}, preserveAspectRatio = true)));
end TestGroundPhysicsRotatedFrame;
