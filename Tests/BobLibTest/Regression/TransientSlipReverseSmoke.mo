within BobLibTest.Regression;

model TransientSlipReverseSmoke

  BobLib.Chassis.Suspension.Tires.MF52.SlipModel.TransientSlip forward;
  BobLib.Chassis.Suspension.Tires.MF52.SlipModel.TransientSlip reverse;

  output Real kappaForward = forward.kappa;
  output Real kappaReverse = reverse.kappa;
  output Real symmetryError = kappaForward + kappaReverse;

equation
  forward.Vx = 10;
  forward.Vy = 0;
  forward.omega = 55;
  forward.R0 = 0.2;
  forward.Fz = 1000;
  forward.gamma = 0;

  reverse.Vx = -10;
  reverse.Vy = 0;
  reverse.omega = -55;
  reverse.R0 = 0.2;
  reverse.Fz = 1000;
  reverse.gamma = 0;

  assert(
    kappaForward > 0,
    "Forward overspeed must produce positive longitudinal slip");
  assert(
    kappaReverse < 0,
    "Reverse overspeed must produce negative longitudinal slip");
  assert(
    abs(symmetryError) < 1e-10,
    "Mirrored wheel kinematics must produce mirrored transient slip");

initial equation
  assert(
    abs(
      -(forward.Vx - forward.R0*forward.omega) -
      sqrt(forward.Vx^2 + forward.V_min^2)*forward.u/forward.sigma_kappa) < 1e-12,
    "Forward transient slip must initialize at the dynamic equilibrium");
  assert(
    abs(
      -(reverse.Vx - reverse.R0*reverse.omega) -
      sqrt(reverse.Vx^2 + reverse.V_min^2)*reverse.u/reverse.sigma_kappa) < 1e-12,
    "Reverse transient slip must initialize at the dynamic equilibrium");

  annotation(
    experiment(StartTime = 0, StopTime = 0.1, Tolerance = 1e-6, Interval = 0.01));
end TransientSlipReverseSmoke;
