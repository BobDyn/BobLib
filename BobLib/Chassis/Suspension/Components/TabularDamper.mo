within BobLib.Chassis.Suspension.Components;

model TabularDamper "Tabular translational damper with velocity-force curve"

  import Modelica.Units.SI;

  extends BobLib.Icons.TabularDamperIcon;

  extends BobLib.Chassis.Suspension.BaseClasses.PartialTabularCompliant;

  // Damper parameters
  parameter Integer damperPoints(min = 2, max = size(damperTable, 1)) = size(damperTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Damper Parameters"));
  parameter Real damperTable[:, 2] "Columns: compression-positive shaft velocity [m/s], signed resisting force [N]" annotation(
    Dialog(group = "Damper Parameters"));
  parameter Boolean useRuntimeTable = false "Read the curve from a file at simulation start"
    annotation(Evaluate = false, choices(checkBox = true), Dialog(group = "Damper Parameters"));
  parameter String runtimeTableFile = "damper_tables.txt" "File containing the runtime force-velocity tables"
    annotation(Evaluate = false, Dialog(group = "Damper Parameters", enable = useRuntimeTable));
  parameter String runtimeTableName = "NoName" "Name of the selected force-velocity table in the runtime file"
    annotation(Dialog(group = "Damper Parameters", enable = useRuntimeTable));

  SI.Velocity v_rel "Rate of increasing damper length; positive in rebound";
  output Boolean extrapolating "True when shaft velocity is outside the active curve";
  output SI.Power dissipatedPower "Mechanical power absorbed by the damper";

  Modelica.Blocks.Sources.RealExpression velExpression(y = -v_rel) annotation(
    Placement(transformation(origin = {-43.5, 45}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Tables.CombiTable1Ds combiTable1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    tableOnFile = useRuntimeTable,
    tableName = if useRuntimeTable then runtimeTableName else "NoName",
    fileName = runtimeTableFile,
    table = BobLib.Utilities.Math.extendLinearTable(damperTable, damperPoints)) annotation(
    Placement(transformation(origin = {20, 45}, extent = {{-10, -10}, {10, 10}})));

initial equation
  assert(useRuntimeTable or (damperPoints >= 2 and damperPoints <= size(damperTable, 1)),
    "Damper active point count must be between two and the table capacity");
  assert(useRuntimeTable or (damperTable[1, 1] <= 0 and damperTable[damperPoints, 1] >= 0),
    "Damper table must include zero velocity");
  assert(useRuntimeTable or sum(if i <= damperPoints and damperTable[i, 1] == 0 and damperTable[i, 2] == 0 then 1 else 0
    for i in 1:size(
      damperTable,
      1)) == 1 or (damperPoints == 2 and abs(damperTable[1, 1]*damperTable[2, 2] - damperTable[2, 1]*damperTable[1, 2]) < 1e-12),
    "Damper curve must pass through zero force at zero velocity");
  for i in 1:size(damperTable, 1) loop
    assert(useRuntimeTable or i > damperPoints or damperTable[i, 1]*damperTable[i, 2] >= 0,
      "Damper force must resist motion");
  end for;
  for i in 2:size(damperTable, 1) loop
    assert(useRuntimeTable or i > damperPoints or damperTable[i, 1] > damperTable[i - 1, 1],
      "Damper active velocities must strictly increase");
  end for;
  assert(useRuntimeTable or (damperTable[2, 2] >= damperTable[1, 2] and damperTable[damperPoints, 2] >= damperTable[damperPoints - 1, 2]),
    "Terminal damper slopes must be nonnegative for passive extrapolation");

equation
  v_rel = der(s_rel);
  extrapolating = noEvent(if useRuntimeTable then -v_rel < combiTable1D.u_min or -v_rel > combiTable1D.u_max
    else -v_rel < damperTable[1, 1] or -v_rel > damperTable[damperPoints, 1]);
  dissipatedPower = -force.f*v_rel;

  connect(velExpression.y, combiTable1D.u) annotation(
    Line(points = {{-32.5, 45}, {7, 45}}, color = {0, 0, 127}));
  connect(combiTable1D.y[1], force.f) annotation(
    Line(points = {{32, 45}, {33, 45}, {33, 16}, {0, 16}, {0, 4}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Documentation(info = "<html>
<p>
Model <code>TabularDamper</code> represents a tabulated damper used by suspension linkage assemblies.
</p>
<p>
Its table data comes from the owning axle or linkage record. The active point count retains fixed-capacity calibration support. Alternatively, enable useRuntimeTable and supply a named table file; the table is loaded when the simulation starts, so curve data can change without recompiling a compatible executable.
</p>
<p>The input is minus the rate of increasing damper length: positive is compression and negative is rebound. The signed force curve uses linear interpolation and terminal-slope extrapolation. Monitor extrapolating and dissipatedPower when validating a curve. The component does not model hysteresis.</p>
<p>With useRuntimeTable the initial checks on damperTable are skipped, so the program that writes the table file owns its validation (BobSim checks passivity and the zero crossing before writing). The embedded damperTable is then only a fallback for runs without the file, and it can differ from the curve the file supplies.</p>
</html>"));
end TabularDamper;
