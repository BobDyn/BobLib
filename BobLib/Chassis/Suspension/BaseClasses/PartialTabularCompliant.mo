within BobLib.Chassis.Suspension.BaseClasses;

partial model PartialTabularCompliant "Base compliant element"

  import Modelica.Units.SI;

  extends Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges;

  // Force generation
  Modelica.Mechanics.Translational.Sources.Force2 force annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

  SI.Position s_rel;

equation

  // Deflection calc
  s_rel = flange_b.s - flange_a.s;

  connect(force.flange_a, flange_a) annotation(
    Line(points = {{-10, 0}, {-100, 0}}, color = {0, 127, 0}));
  connect(force.flange_b, flange_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 127, 0}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Documentation(info = "<html>
<p>
Model <code>PartialTabularCompliant</code> represents a shared compliant-line template used by suspension linkage assemblies.
</p>
<p>
Its table data comes from the owning axle or linkage record, allowing measured or tuned force curves to be changed without changing the multibody connection topology.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)));
end PartialTabularCompliant;
