within BobLib.Chassis.Suspension.Linkages;

model TabularDamper "Tabular translational damper with velocity-force curve"

  extends BobLib.Icons.TabularDamperIcon;

  import SI = Modelica.Units.SI;

  extends BobLib.Chassis.Suspension.Linkages.Templates.TabularCompliant;

  // Damper parameters
  parameter Real damperTable[:, 2] "Columns: compression-positive shaft velocity [m/s], resisting force [N]" annotation(
    Dialog(group = "Damper Parameters"));

  SI.Velocity v_rel "Rate of increasing shock length; positive in rebound";
  Real v_abs;
  Real vel_sgn;
  output Boolean extrapolating "True outside the supplied velocity range";
  output SI.Power dissipatedPower "Power absorbed by the damper";

  // Velocity processing blocks
  Modelica.Blocks.Sources.RealExpression velExpression(y = v_abs) annotation(
    Placement(transformation(origin = {-90, 36}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression sgnExpression(y = -vel_sgn) annotation(
    Placement(transformation(origin = {-90, 24}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Product product annotation(
    Placement(transformation(origin = {-60, 30}, extent = {{-10, -10}, {10, 10}})));

  // Force output block
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = damperTable) annotation(
    Placement(transformation(origin = {-20, 30}, extent = {{-10, -10}, {10, 10}})));

initial equation
  assert(size(damperTable, 1) >= 3, "Damper table needs rebound, zero and compression rows");
  assert(damperTable[1, 1] < 0 and damperTable[end, 1] > 0,
    "Damper table must cover both compression and rebound");
  assert(sum(if abs(damperTable[i, 1]) <= 0 and abs(damperTable[i, 2]) <= 0 then 1 else 0
    for i in 1:size(damperTable, 1)) == 1, "Damper table must include one [0, 0] row");
  for i in 1:size(damperTable, 1) loop
    assert(damperTable[i, 1]*damperTable[i, 2] >= 0, "Damper force must resist motion");
  end for;
  for i in 2:size(damperTable, 1) loop
    assert(damperTable[i, 1] > damperTable[i - 1, 1], "Damper velocities must strictly increase");
  end for;
  assert(damperTable[2, 2] >= damperTable[1, 2] and damperTable[end, 2] >= damperTable[end - 1, 2],
    "Terminal slopes must be nonnegative for passive extrapolation");

equation
  v_rel = der(s_rel);
  v_abs = sqrt(v_rel*v_rel + eps*eps);
  vel_sgn = v_rel/v_abs;
  extrapolating = noEvent(-v_rel < damperTable[1, 1] or -v_rel > damperTable[end, 1]);
  dissipatedPower = -force.f*v_rel;

  connect(velExpression.y, product.u1) annotation(
    Line(points = {{-79, 36}, {-73, 36}}, color = {0, 0, 127}));
  connect(sgnExpression.y, product.u2) annotation(
    Line(points = {{-79, 24}, {-73, 24}}, color = {0, 0, 127}));
  connect(product.y, combiTable1D.u) annotation(
    Line(points = {{-49, 30}, {-33, 30}}, color = {0, 0, 127}));
  connect(combiTable1D.y[1], force.f) annotation(
    Line(points = {{-8, 30}, {0, 30}, {0, 4}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Documentation(info = "<html>
<p>
Model <code>TabularDamper</code> represents a tabulated damper used by suspension linkage assemblies.
</p>
<p>
Its table data comes from the owning axle or linkage record, allowing measured or tuned force curves to be changed without changing the multibody connection topology.
</p>
<p>The lookup input is minus the rate of increasing shock length: positive is
compression, negative is rebound. The table stores signed resisting force,
with a continuous zero-force origin. It uses linear segments and terminal-slope
extrapolation. Monitor <code>extrapolating</code> to identify operation outside
the supplied data. Settings and measured-curve provenance are resolved by BobSim
before generating the vehicle record; this component does not model hysteresis.</p>
</html>"));
end TabularDamper;
