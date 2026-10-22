within BobLibTest.Chassis.Suspension.Components;
model TestBellcrank2Motion "Analytic constant-speed bellcrank pickup trajectories"

  extends TestBellcrank2(bellcrank(revolute(
    phi(start = 0, fixed = true), w(start = 2, fixed = true))));

equation

  // Gravity has no moment about the vertical pivot; all pickups rotate at 2 rad/s.
  assert(max(abs(bellcrank.pickupFrame1.r_0 -
    {0.1*cos(2*time), 0.1*sin(2*time), 0})) < 1e-7,
    "First bellcrank pickup must follow its analytic circular trajectory");
  assert(max(abs(bellcrank.pickupFrame2.r_0 -
    {0.1*(cos(2*time)-sin(2*time)), 0.1*(sin(2*time)+cos(2*time)), 0})) < 1e-7,
    "Second bellcrank pickup must preserve its phase and radius");
  annotation(experiment(StopTime = 1, Tolerance = 1e-8, Interval = 0.01),
    Documentation(info = "<html><p>Checks each external pickup position against an independent constant-speed rotation solution. Inherits the component fixture diagram and leaves its initialization baseline unchanged.</p></html>"));
end TestBellcrank2Motion;
