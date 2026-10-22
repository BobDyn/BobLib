within BobLibTest.Drivelines;
model TestRearFinalDriveDifferential

  BobLib.Drivelines.RearFinalDriveDifferential dut annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque drive(tau_constant = 100) annotation(Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia left(
    J = 1,
    phi(start = 0, fixed = true),
    w(
      start = 0,
      fixed = true)) annotation(Placement(transformation(origin = {60, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia right(
    J = 1,
    phi(start = 0, fixed = true),
    w(
      start = 0,
      fixed = true)) annotation(Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(drive.flange, dut.transmissionFlange.flange) annotation(Line(points = {{-50, 0}, {-20, 0}}, color = {0, 0, 0}));
  connect(dut.wheelHub_3.flange, left.flange_a) annotation(Line(points = {{12, -20}, {12, -40}, {50, -40}}, color = {0, 0, 0}));
  connect(dut.wheelHub_4.flange, right.flange_a) annotation(Line(points = {{12, 20}, {12, 40}, {50, 40}}, color = {0, 0, 0}));
  assert(abs(left.w-right.w)<1e-7, "Symmetric loads must accelerate together");
  assert(abs(dut.leftHalfshaftTorque-dut.rightHalfshaftTorque)<1e-6, "Symmetric halfshaft torques");
  assert(abs(dut.motorSideSpeed-dut.pDriveline.inputRatio*dut.diffInputSpeed)<1e-8, "Final-drive speed ratio");

  when terminal() then
    assert(left.w>0, "Positive input torque must accelerate the wheels");
  end when;
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-120, -80}, {120, 80}}, preserveAspectRatio = true)));
end TestRearFinalDriveDifferential;
