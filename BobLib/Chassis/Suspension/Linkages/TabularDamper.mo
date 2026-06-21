within BobLib.Chassis.Suspension.Linkages;

model TabularDamper "Tabular translational damper with velocity-force curve"

  extends BobLib.Icons.TabularDamperIcon;

  import SI = Modelica.Units.SI;

  extends BobLib.Chassis.Suspension.Linkages.Templates.TabularCompliant;

  // Damper parameters
  parameter SI.TranslationalDampingConstant damperTable[:, 2] "Table of Force vs Relative Velocity (m/s, N)" annotation(
    Dialog(group = "Damper Parameters"));

  Real v_rel;
  Real v_abs;
  Real vel_sgn;

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

equation
  v_rel = der(s_rel);
  v_abs = sqrt(v_rel*v_rel + eps*eps);
  vel_sgn = v_rel/v_abs;

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
</html>"));
end TabularDamper;
