within BobLibTest.Utilities.Math;

model TestExtendLinearTable

  parameter Integer activePoints = 2 "Number of valid leading rows in the fixed-capacity interpolation table" annotation(Dialog(group = "Operating settings"), Evaluate = false);
  Real extended[4, 2];

equation
  extended = BobLib.Utilities.Math.extendLinearTable(
    [0, 0; 1, 10; 2, 30; -999, 999], activePoints);
  assert(abs(extended[4, 1] - 3) < 1e-12, "Unused breakpoints were not replaced");
  assert(abs(extended[4, 2] - (if activePoints == 2 then 30 else 50)) < 1e-12,
    "Active count did not change the final extrapolation segment");
  annotation(experiment(StopTime = 0.01, Interval = 0.01));
end TestExtendLinearTable;
