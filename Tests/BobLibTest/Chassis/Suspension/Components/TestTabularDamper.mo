within BobLibTest.Chassis.Suspension.Components;

model TestTabularDamper

  BobLib.Chassis.Suspension.Components.TabularDamper dut(damperTable = [-1, -100; 0, 0; 1, 100]) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Sources.Position motion(
    exact = true,
    s_ref = 0.5 + 0.01*sin(100*time)) annotation(Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

equation
  connect(fixed.flange, dut.flange_a) annotation(Line(points = {{-60, 0}, {-10, 0}}, color = {0, 0, 0}));
  connect(dut.flange_b, motion.flange) annotation(Line(points = {{10, 0}, {50, 0}}, color = {0, 0, 0}));
  assert(abs(dut.flange_a.f + dut.flange_b.f) < 1e-8, "Equal and opposite forces");
  assert(abs(dut.force.f + 100*cos(100*time)) < 1e-6, "Signed damping force through positive, negative and zero speed");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-100, -40}, {100, 40}}, preserveAspectRatio = true)));
end TestTabularDamper;
