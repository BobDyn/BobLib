within BobLibTest.Chassis.Suspension.Components;

model TestShockLinkagePreload

  extends BobLibTest.Chassis.Suspension.Components.TestShockLinkage(
    fixedB(r = {1.02, 0, 0}));

equation
  assert(abs(shock.frame_a.f[1] + 20) < 1e-6 and
    abs(shock.frame_b.f[1] - 20) < 1e-6,
    "Prescribed extension must produce equal-and-opposite tensile reactions");
  annotation(Documentation(info = "<html><p>Checks the external frame reactions against the independently calculated force at a prescribed static extension.</p></html>"));
end TestShockLinkagePreload;
