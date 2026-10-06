within BobLibTest.Regression;

model UniversalSphericalReferenceSmoke

  "Compare the local derived reference frame with MSL under non-axis-aligned motion"

  inner Modelica.Mechanics.MultiBody.World world(g = 0, enableAnimation = false);

  Modelica.Mechanics.MultiBody.Joints.Revolute pivot(n = {1, 0.3, 0.1}, useAxisFlange = true);

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tip(r = {0.15, 0.4, 0.3});

  Modelica.Mechanics.Rotational.Sources.Position drive(exact = true);

  BobLib.Utilities.Mechanics.MultiBody.Joints.Internal.UniversalSpherical localRod(
    n1_a = {0.2, 0.1, 1}, rRod_ia = {0.15, 0.4, 0.3},
    kinematicConstraint = false, constraintResidue = localRod.f_rod);

  Modelica.Mechanics.MultiBody.Joints.UniversalSpherical referenceRod(
    n1_a = {0.2, 0.1, 1}, rRod_ia = {0.15, 0.4, 0.3},
    kinematicConstraint = false, constraintResidue = referenceRod.f_rod);
  output Real rotationError = max(abs(localRod.frame_ia.R.T - referenceRod.frame_ia.R.T));
  output Real angularVelocityError = Modelica.Math.Vectors.norm(localRod.frame_ia.R.w - referenceRod.frame_ia.R.w);

equation
  connect(world.frame_b, pivot.frame_a);
  connect(pivot.frame_b, tip.frame_a);
  connect(drive.flange, pivot.axis);
  drive.phi_ref = 0.2*sin(time);
  connect(world.frame_b, localRod.frame_a);
  connect(world.frame_b, referenceRod.frame_a);
  connect(tip.frame_b, localRod.frame_b);
  connect(tip.frame_b, referenceRod.frame_b);
  assert(rotationError < 1e-10, "Derived reference frame changes the joint orientation");
  assert(angularVelocityError < 1e-10, "Derived reference frame changes angular velocity");

  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-8, Interval = 0.01));
end UniversalSphericalReferenceSmoke;
