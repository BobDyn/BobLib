within BobLibTest.Brakes;
model TestBasicVCUBrakes

  BobLib.Brakes.BasicVCUBrakes dut(
    pBrakes(maxTorque = 1000, frontBrakeBias = 0.6),
    usingMultiBodyChassis = false) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.Constant command(k = 2000) annotation(Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed wheel1(
    exact = true,
    w_ref = 10) annotation(Placement(transformation(origin = {-50, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.Rotational.Sources.Speed wheel2(
    exact = true,
    w_ref = -10) annotation(Placement(transformation(origin = {-50, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Mechanics.Rotational.Sources.Speed wheel3(
    exact = true,
    w_ref = 10) annotation(Placement(transformation(origin = {50, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.Rotational.Sources.Speed wheel4(
    exact = true,
    w_ref = -10) annotation(Placement(transformation(origin = {50, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  inner Modelica.Mechanics.MultiBody.World world annotation(
    Placement(transformation(origin = {-130, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(command.y, dut.controlBus.brakesControlBus.mechanicalBrakeTorqueRequest) annotation(Line(points = {{-69, 0}, {-67, 0}, {-67, 12}, {-20, 12}}, color = {0, 0, 127}));
  connect(wheel1.flange, dut.wheelHub_1.flange) annotation(Line(points = {{-50, -35}, {-50, -24}, {-12, -24}, {-12, -20}}, color = {0, 0, 0}));
  connect(wheel2.flange, dut.wheelHub_2.flange) annotation(Line(points = {{-50, 35}, {-50, 26}, {-12, 26}, {-12, 20}}, color = {0, 0, 0}));
  connect(wheel3.flange, dut.wheelHub_3.flange) annotation(Line(points = {{50, -35}, {50, -24}, {12, -24}, {12, -20}}, color = {0, 0, 0}));
  connect(wheel4.flange, dut.wheelHub_4.flange) annotation(Line(points = {{50, 35}, {50, 26}, {12, 26}, {12, 20}}, color = {0, 0, 0}));
  assert(abs(dut.mechanicalBrakeTorqueRequest-1000)<1e-8, "Brake request must be limited");
  assert(abs(dut.brakeTorque_1-300)<1e-8 and abs(dut.brakeTorque_3-200)<1e-8, "Front/rear hydraulic bias");
  assert(
    abs(dut.wheelHub_1.flange.tau-300)<1e-8 and abs(dut.wheelHub_2.flange.tau+300)<1e-8,
    "Brakes oppose both forward and reverse rotation");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-140, -80}, {100, 80}}, preserveAspectRatio = true)));
end TestBasicVCUBrakes;
