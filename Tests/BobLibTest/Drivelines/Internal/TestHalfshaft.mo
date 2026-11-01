within BobLibTest.Drivelines.Internal;
model TestHalfshaft "Halfshaft acceleration and torque balance"

  BobLib.Drivelines.Internal.Halfshaft dut(fixInitialSpeed = true) annotation(
    Placement(transformation(extent = {{-15, -15}, {15, 15}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque drive(tau_constant = 10) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia load(J = 1, phi(fixed = true), w(fixed = true)) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(drive.flange, dut.flange_a) annotation(Line(points = {{-50, 0}, {-15, 0}}));
  connect(dut.flange_b, load.flange_a) annotation(Line(points = {{15, 0}, {50, 0}}));
  assert(abs(dut.pHalfshaft.inertia*der(dut.rotor.w) + load.J*der(load.w) - 10) < 1e-7,
    "Halfshaft and wheel angular momentum must balance applied torque");
  assert(abs(dut.tau+load.flange_a.tau) < 1e-8, "Halfshaft output torque must reach the wheel");

  when terminal() then
    assert(load.w > 0, "Driven halfshaft must accelerate the wheel");
  end when;
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-8),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}}, preserveAspectRatio = true)));
end TestHalfshaft;
