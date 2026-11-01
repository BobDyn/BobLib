within BobLib.Chassis.Suspension.Components;

model TabularSpring "Tabular translational spring"

  import Modelica.Units.SI;

  extends BobLib.Icons.TabularSpringIcon;

  extends BobLib.Chassis.Suspension.BaseClasses.PartialTabularCompliant;

  // Spring parameters
  parameter Integer springPoints(min = 2, max = size(springTable, 1)) = size(springTable, 1)
    "Number of active curve points" annotation(Evaluate = false, Dialog(group = "Spring Parameters"));
  parameter Real springTable[:, 2] "Table of Force vs Compression (dx, F)" annotation(
    Evaluate = false, Dialog(group = "Spring Parameters"));
  parameter SI.Length s_0 "Free length of spring" annotation(
    Evaluate = false, Dialog(group = "Spring Parameters"));

  SI.Position defl;

  Modelica.Blocks.Sources.RealExpression deflExpression(y = defl) annotation(
    Placement(transformation(origin = {-43.5, 45}, extent = {{-10, -10}, {10, 10}})));

  // Force output block
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = BobLib.Utilities.Math.extendLinearTable(springTable, springPoints)) annotation(
    Placement(transformation(origin = {20, 45}, extent = {{-10, -10}, {10, 10}})));

equation
  defl = s_0 - s_rel;

  connect(deflExpression.y, combiTable1D.u) annotation(
    Line(points = {{-32.5, 45}, {7, 45}}, color = {0, 0, 127}));
  connect(combiTable1D.y[1], force.f) annotation(
    Line(points = {{32, 45}, {33, 45}, {33, 16}, {0, 16}, {0, 4}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Documentation(info = "<html>
<p>
Model <code>TabularSpring</code> represents a tabulated spring used by suspension linkage assemblies.
</p>
<p>
Its table data comes from the owning axle or linkage record, allowing measured or tuned force curves to be changed without changing the multibody connection topology.
</p>
</html>"));
end TabularSpring;
