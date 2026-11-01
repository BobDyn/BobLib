within BobLibTest.Brakes.Internal;
model TestAxleBrakes "Opposed rotor directions and independent spatial caliper reactions"

  inner Modelica.Mechanics.MultiBody.World world(
    g = 0,
    enableAnimation = false,
    driveTrainMechanics3D = true) annotation(Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Brakes.Internal.AxleBrakes dut(
    usingMultiBodyChassis = true,
    leftTorqueRequest = 100,
    rightTorqueRequest = 140) annotation(Placement(transformation(origin = {0, 20}, extent = {{-40, -20}, {40, 20}})));

  Modelica.Mechanics.Rotational.Sources.Speed leftSpeed(
    exact = true,
    w_ref = 10) annotation(Placement(transformation(origin = {-80, 28}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed rightSpeed(
    exact = true,
    w_ref = -10) annotation(Placement(transformation(origin = {80, 28}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed leftUpright annotation(Placement(transformation(origin = {-80, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed rightUpright annotation(Placement(transformation(origin = {80, -20}, extent = {{10, -10}, {-10, 10}})));

equation
  connect(leftSpeed.flange, dut.leftRotor) annotation(Line(points = {{-70, 28}, {-40, 28}}));
  connect(rightSpeed.flange, dut.rightRotor) annotation(Line(points = {{70, 28}, {40, 28}}));
  connect(leftUpright.frame_b, dut.leftCaliperMount) annotation(Line(points = {{-70, -20}, {-40, -20}, {-40, 12}}, color = {95, 95, 95}));
  connect(rightUpright.frame_b, dut.rightCaliperMount) annotation(Line(points = {{70, -20}, {40, -20}, {40, 12}}, color = {95, 95, 95}));
  assert(
    abs(dut.leftRotor.tau-100)<1e-8 and abs(dut.rightRotor.tau+140)<1e-8,
    "Each brake opposes its own rotor direction");
  assert(
    abs(dut.leftCaliperMount.t[2]+100)<1e-8 and abs(dut.rightCaliperMount.t[2]-140)<1e-8,
    "Each caliper carries its rotor's opposite reaction independently");
  annotation(experiment(StopTime = 0.1), Diagram(coordinateSystem(extent = {{-120, -80}, {120, 80}})));
end TestAxleBrakes;
