within BobLibTest.Utilities.Mechanics.MultiBody.Joints.Internal;

model UniversalSphericalReferenceSmoke

  "Compare the local derived reference frame with MSL under non-axis-aligned motion"

  inner Modelica.Mechanics.MultiBody.World world(
    g = 0,
    enableAnimation = false) annotation(Placement(transformation(origin = {-250, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Joints.Revolute pivot(
    n = {1, 0.3, 0.1},
    useAxisFlange = true) annotation(Placement(transformation(origin = {-60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tip(r = {0.15, 0.4, 0.3}) annotation(Placement(transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Position drive(exact = true) annotation(Placement(transformation(origin = {-60, 140}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.Utilities.Mechanics.MultiBody.Joints.Internal.UniversalSpherical localRod(
    n1_a = {0.2, 0.1, 1}, rRod_ia = {0.15, 0.4, 0.3},
    kinematicConstraint = false, constraintResidue = localRod.f_rod) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Joints.UniversalSpherical referenceRod(
    n1_a = {0.2, 0.1, 1}, rRod_ia = {0.15, 0.4, 0.3},
    kinematicConstraint = false, constraintResidue = referenceRod.f_rod) annotation(Placement(transformation(origin = {0, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  output Real rotationError = max(abs(localRod.frame_ia.R.T - referenceRod.frame_ia.R.T));
  output Real angularVelocityError = Modelica.Math.Vectors.norm(localRod.frame_ia.R.w - referenceRod.frame_ia.R.w);

equation
  connect(world.frame_b, pivot.frame_a) annotation(Line(points = {{-240, -110}, {-180, -110}, {-180, 80}, {-70, 80}}, color = {95, 95, 95}));
  connect(pivot.frame_b, tip.frame_a) annotation(Line(points = {{-50, 80}, {50, 80}}, color = {95, 95, 95}));
  connect(drive.flange, pivot.axis) annotation(Line(points = {{-50, 140}, {-47, 140}, {-47, 96}, {-60, 96}, {-60, 90}}, color = {0, 0, 0}));
  drive.phi_ref = 0.2*sin(time);
  connect(world.frame_b, localRod.frame_a) annotation(Line(points = {{-240, -110}, {-180, -110}, {-180, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(world.frame_b, referenceRod.frame_a) annotation(Line(points = {{-240, -110}, {-180, -110}, {-180, -80}, {-20, -80}}, color = {95, 95, 95}));
  connect(tip.frame_b, localRod.frame_b) annotation(Line(points = {{70, 80}, {73, 80}, {73, 0}, {20, 0}}, color = {95, 95, 95}));
  connect(tip.frame_b, referenceRod.frame_b) annotation(Line(points = {{70, 80}, {77, 80}, {77, -80}, {20, -80}}, color = {95, 95, 95}));
  assert(rotationError < 1e-10, "Derived reference frame changes the joint orientation");
  assert(angularVelocityError < 1e-10, "Derived reference frame changes angular velocity");

  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-8, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-260, -120}, {180, 180}}, preserveAspectRatio = true)));
end UniversalSphericalReferenceSmoke;
