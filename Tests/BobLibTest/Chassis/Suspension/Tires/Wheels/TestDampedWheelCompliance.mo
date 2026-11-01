within BobLibTest.Chassis.Suspension.Tires.Wheels;
model TestDampedWheelCompliance

  inner Modelica.Mechanics.MultiBody.World world(
    g = 0,
    enableAnimation = false) annotation(Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed support(animation = false) annotation(Placement(transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.Wheels.DampedWheelCompliance dut(
    slideX(s(start = 0, fixed = true)),
    slideY(s(start = 0, fixed = true)),
    slideZ(s(start = 0, fixed = true)),
    rotateX(phi(start = 0, fixed = true)),
    rotateY(phi(start = 0, fixed = true)),
    rotateZ(phi(start = 0, fixed = true))) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque load(
    animation = false,
    force = {0, 0, 100},
    torque = {0, 0, 10},
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}})));

equation
  connect(support.frame_b, dut.frame_a) annotation(Line(points = {{-80, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(dut.frame_b, load.frame_b) annotation(Line(points = {{20, 0}, {80, 0}}, color = {95, 95, 95}));
  assert(
    abs(dut.slideZ.s - 100/dut.pCompliance.translationStiffness[3]*(1-exp(-time*dut.pCompliance.translationStiffness[3]/dut.pCompliance.translationDamping[3])))<1e-8,
    "Translation must match the analytic damped relaxation");
  assert(
    abs(dut.rotateZ.phi - 10/dut.pCompliance.rotationStiffness[3]*(1-exp(-time*dut.pCompliance.rotationStiffness[3]/dut.pCompliance.rotationDamping[3])))<1e-8,
    "Rotation must match the analytic damped relaxation");
  annotation(experiment(StopTime = 0.01, Tolerance = 1e-9), Diagram(coordinateSystem(extent = {{-120, -80}, {120, 80}})));
end TestDampedWheelCompliance;
