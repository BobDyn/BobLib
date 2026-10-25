within BobLib.Chassis.Suspension.Components;

model TabularDamper "Tabular translational damper with velocity-force curve"

  import Modelica.Units.SI;

  extends BobLib.Icons.TabularDamperIcon;

  extends BobLib.Chassis.Suspension.BaseClasses.PartialTabularCompliant;

  // Damper parameters
  parameter Integer damperPoints(min = 2, max = size(damperTable, 1)) = size(damperTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Damper Parameters"));
  parameter Real damperTable[:, 2] "Table of Force vs Relative Velocity (m/s, N)" annotation(
    Dialog(group = "Damper Parameters"));

  SI.Velocity v_rel;

  Modelica.Blocks.Sources.RealExpression velExpression(y = -v_rel) annotation(
    Placement(transformation(origin = {-43.5, 45}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Tables.CombiTable1Ds combiTable1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = BobLib.Utilities.Math.extendLinearTable(damperTable, damperPoints)) annotation(
    Placement(transformation(origin = {20, 45}, extent = {{-10, -10}, {10, 10}})));

equation
  v_rel = der(s_rel);

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
Its table data comes from the owning axle or linkage record, allowing measured or tuned force curves to be changed without changing the multibody connection topology.
</p>
</html>"));
end TabularDamper;
