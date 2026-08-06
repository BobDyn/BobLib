within BobLibTest.Regression;

model AngleBetweenSmoke

  output Real tinyAntiparallel =
    BobLib.Utilities.Math.Vector.angle_between(
      {1e-12, 0, 0},
      {-1e-12, 0, 0},
      {0, 0, 1}) + 0*time;

  output Real scaledQuarterTurn =
    BobLib.Utilities.Math.Vector.angle_between(
      {1e-20, 0, 0},
      {0, 1e20, 0},
      {0, 0, 1e-9}) + 0*time;

  output Real negativeQuarterTurn =
    BobLib.Utilities.Math.Vector.angle_between(
      {1, 0, 0},
      {0, -1, 0},
      {0, 0, 1}) + 0*time;

  output Real zeroVectorAngle =
    BobLib.Utilities.Math.Vector.angle_between(
      {0, 0, 0},
      {1, 0, 0},
      {0, 0, 1}) + 0*time;

equation
  assert(
    abs(tinyAntiparallel - Modelica.Constants.pi) < 1e-12,
    "Tiny antiparallel vectors must retain their pi angle");
  assert(
    abs(scaledQuarterTurn - Modelica.Constants.pi/2) < 1e-12,
    "Angle must be invariant to vector and axis magnitude");
  assert(
    abs(negativeQuarterTurn + Modelica.Constants.pi/2) < 1e-12,
    "Signed negative rotation must be preserved");
  assert(
    abs(zeroVectorAngle) < 1e-12,
    "Zero-length inputs must use the documented neutral result");

  annotation(
    experiment(StartTime = 0, StopTime = 0.001, Tolerance = 1e-6, Interval = 0.001));
end AngleBetweenSmoke;
