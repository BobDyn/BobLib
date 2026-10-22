within BobLib.Utilities.Math;

pure function extendLinearTable "Ignore unused rows while preserving linear extrapolation"
  input Real table[:, 2];
  input Integer activePoints = size(table, 1);
  output Real extended[size(table, 1), 2];

algorithm
  assert(activePoints >= 2 and activePoints <= size(table, 1),
    "Active curve points must be between two and the table capacity");
  for i in 2:activePoints loop
    assert(table[i, 1] > table[i - 1, 1],
      "Active curve breakpoints must be strictly increasing");
  end for;
  extended := table;

  // MSL tables retain fixed dimensions; continue the last active segment.
  for i in activePoints + 1:size(table, 1) loop
    extended[i, :] := table[activePoints, :] +
      (i - activePoints)*(table[activePoints, :] - table[activePoints - 1, :]);
  end for;
  annotation(Inline = false, Documentation(info = "<html><p>Copies active curve rows and extends their last segment into unused capacity. This preserves linear interpolation and extrapolation without changing table dimensions. Unused input rows are ignored.</p></html>"));
end extendLinearTable;
