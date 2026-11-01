within BobLibTest.Chassis.Suspension.Tires.Wheels;
model TestWheelCompliance "Six-axis elastic wheel-stack equilibrium and caliper-side reaction"

  parameter Integer torqueAxis(
    min = 1,
    max = 3) = 1 "Single local axis loaded by the torque probe" annotation(Evaluate = true);

  inner Modelica.Mechanics.MultiBody.World world(
    g = 0,
    enableAnimation = false) annotation(Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed support(animation = false) annotation(Placement(transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.Wheels.WheelCompliance dut annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque load(
    animation = false,
    force = {100, 200, 300},
    torque = {if i == torqueAxis then 10 else 0 for i in 1:3},
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}})));

equation
  connect(support.frame_b, dut.frame_a) annotation(Line(points = {{-80, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(dut.frame_b, load.frame_b) annotation(Line(points = {{20, 0}, {80, 0}}, color = {95, 95, 95}));
  assert(abs(dut.slideX.s - 100/dut.pCompliance.translationStiffness[1])<1e-10, "X translation must follow F/k");
  assert(abs(dut.slideY.s - 200/dut.pCompliance.translationStiffness[2])<1e-10, "Y translation must follow F/k");
  assert(abs(dut.slideZ.s - 300/dut.pCompliance.translationStiffness[3])<1e-10, "Z translation must follow F/k");
  assert(
    sum(({dut.rotateX.phi, dut.rotateY.phi, dut.rotateZ.phi} - {if i == torqueAxis then 10/dut.pCompliance.rotationStiffness[i] else 0 for i in 1:3}).^2)<1e-18,
    "Loaded rotation must follow torque/stiffness and unloaded rotations must remain neutral");
  assert(sum((support.frame_b.f - {100, 200, 300}).^2)<1e-12, "Support must carry the applied force");
  annotation(experiment(StopTime = 0.1), Diagram(coordinateSystem(extent = {{-120, -80}, {120, 80}})));
end TestWheelCompliance;
