within BobLibTest.Chassis.Suspension.Components;

model TestForceOnlyRodPreload

  extends BobLibTest.Chassis.Suspension.Components.TestForceOnlyRod(
    fixedB(r = {1.001, 0, 0}),
    rod(springDamper(s_rel(start = 1.001))));

equation
  assert(abs(rod.frame_a.f[1] + 8600) < 1e-6 and
    abs(rod.frame_b.f[1] - 8600) < 1e-6,
    "Prescribed extension must produce equal-and-opposite tensile reactions");
  annotation(Documentation(info = "<html><p>Checks the external frame reactions against the independently calculated force at a prescribed static extension.</p></html>"));
end TestForceOnlyRodPreload;
