within BobLibTest.Brakes;
model TestPedalBrakes

  parameter Modelica.Units.SI.Position travel = 0.025 "Initial brake pedal displacement" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Velocity travelRate = 0 "Prescribed rate of brake pedal displacement" annotation(Dialog(group = "Operating settings"));

  inner Modelica.Mechanics.MultiBody.World world(g = 0, enableAnimation = false, driveTrainMechanics3D = true)
    annotation(Placement(transformation(origin = {-110, -90}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Brakes.PedalBrakes dut(
    pBrakes(maxTorque = 1000, pedalFullTravel = 0.05, pedalStiffness = 20000, pedalDamping = 100),
    usingMultiBodyChassis = true)
    annotation(Placement(transformation(origin = {20, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.Translational.Sources.Position pedal(exact = true, s_ref = travel+travelRate*time)
    annotation(Placement(transformation(origin = {-40, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Speed wheels[4](each exact = true, w_ref = {10, -10, 10, -10})
    annotation(Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(pedal.flange, dut.brakePedal) annotation(Line(points = {{-30, 70}, {20, 70}, {20, 20}}, color = {0, 127, 0}));
  connect(wheels[1].flange, dut.wheelHub_1.flange) annotation(Line(points = {{-70, 0}, {-50, 0}, {-50, -40}, {8, -40}, {8, -20}}));
  connect(wheels[2].flange, dut.wheelHub_2.flange) annotation(Line(points = {{-70, 0}, {-50, 0}, {-50, 40}, {8, 40}, {8, 20}}));
  connect(wheels[3].flange, dut.wheelHub_3.flange) annotation(Line(points = {{-70, 0}, {-60, 0}, {-60, -50}, {32, -50}, {32, -20}}));
  connect(wheels[4].flange, dut.wheelHub_4.flange) annotation(Line(points = {{-70, 0}, {-60, 0}, {-60, 50}, {32, 50}, {32, 20}}));
  connect(world.frame_b, dut.wheelHub_1.bearingFrame) annotation(Line(points = {{-100, -90}, {8, -90}, {8, -20}}, color = {95, 95, 95}));
  connect(world.frame_b, dut.wheelHub_2.bearingFrame) annotation(Line(points = {{-100, -90}, {60, -90}, {60, 60}, {8, 60}, {8, 20}}, color = {95, 95, 95}));
  connect(world.frame_b, dut.wheelHub_3.bearingFrame) annotation(Line(points = {{-100, -90}, {32, -90}, {32, -20}}, color = {95, 95, 95}));
  connect(world.frame_b, dut.wheelHub_4.bearingFrame) annotation(Line(points = {{-100, -90}, {70, -90}, {70, 60}, {32, 60}, {32, 20}}, color = {95, 95, 95}));
  assert(
    abs(dut.mechanicalBrakeTorqueRequest-1000*min(1, max(0, (travel+travelRate*time)/0.05)))<1e-9,
    "Pedal demand is proportional and bounded");
  assert(abs(dut.brakePedal.f-(20000*(travel+travelRate*time)+100*travelRate))<1e-9, "Pedal compliance force");
  assert(abs(dut.wheelHub_1.bearingFrame.t[2]+dut.wheelHub_1.flange.tau)<1e-9, "Caliper reaction");
  assert(dut.wheelHub_1.flange.tau*der(dut.wheelHub_1.flange.phi)>=0, "Brake absorbs rotor power");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-120, -100}, {100, 100}})));
end TestPedalBrakes;
