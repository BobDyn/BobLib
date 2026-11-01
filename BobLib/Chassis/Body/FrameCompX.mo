within BobLib.Chassis.Body;

model FrameCompX

  extends BobLib.Chassis.Body.FrameBase;

  Modelica.Mechanics.MultiBody.Joints.Revolute torsionalRevolute(
    n = {1, 0, 0},
    useAxisFlange = true,
    stateSelect = StateSelect.always,
    phi(nominal = 1e-4),
    w(start = 0, fixed = true, nominal = 1)) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -180)));

  replaceable Modelica.Mechanics.Rotational.Components.Spring spring(
    c = pFrame.torsionalStiffness,
    phi_rel0 = 0) constrainedby Modelica.Mechanics.Rotational.Interfaces.PartialCompliant annotation(choicesAllMatching = true,
    Placement(transformation(origin = {20, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  replaceable Modelica.Mechanics.Rotational.Components.Damper damper(d = pFrame.torsionalDamping) constrainedby Modelica.Mechanics.Rotational.Interfaces.PartialCompliant annotation(choicesAllMatching = true,
    Placement(transformation(origin = {20, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

equation
  connect(midToAft.frame_a, torsionalRevolute.frame_a) annotation(
    Line(points = {{40, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(midToFore.frame_a, torsionalRevolute.frame_b) annotation(
    Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(spring.flange_a, torsionalRevolute.support) annotation(
    Line(points = {{30, -30}, {40, -30}, {40, -10}, {6, -10}}));
  connect(spring.flange_b, torsionalRevolute.axis) annotation(
    Line(points = {{10, -30}, {0, -30}, {0, -10}}));
  connect(damper.flange_a, torsionalRevolute.support) annotation(
    Line(points = {{30, -50}, {40, -50}, {40, -10}, {6, -10}}));
  connect(damper.flange_b, torsionalRevolute.axis) annotation(
    Line(points = {{10, -50}, {0, -50}, {0, -10}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -80}, {100, 80}}, preserveAspectRatio = true)),
    Icon(graphics = {Rectangle(extent = {{-21, 9}, {21, -9}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(origin = {0.03, -1}, rotation = -90, points = {{-1.02758, -20}, {-1.02758, -6}, {-5.02758, -4}, {2.97242, 0}, {-5.02758, 4}, {2.97242, 8}, {-1.02758, 10}, {-1.02758, 20}}, color = {0, 170, 0}, thickness = 2)}),
    Documentation(info = "<html>
<p>
Model <code>FrameCompX</code> represents a sprung chassis frame with longitudinal compliance.
</p>
<p>
It is useful when studying chassis compliance effects while keeping the same frame interface as the rigid body model.
</p>
</html>"));
end FrameCompX;
