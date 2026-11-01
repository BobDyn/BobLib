within BobLibTest.Utilities.Mechanics.MultiBody.Actuators;
model TestContactPatchForceActuator

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    g = 9.81,
    enableAnimation = false) annotation(Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed support(
    r = {0, 0, 0},
    animation = false) annotation(Placement(transformation(origin = {60, 0}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator dut(
    fxTable = [0, 0;1, 1],
    fyTable = [0, 0;1, -1],
    forceMagnitude = 100) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(dut.chassisFrame, support.frame_b) annotation(Line(points = {{20, 0}, {50, 0}}, color = {95, 95, 95}));
  assert(
    abs(dut.chassisFrame.f[1]+100*min(time, 1))<1e-8 and abs(dut.chassisFrame.f[2]-100*min(time, 1))<1e-8,
    "Tabulated longitudinal and lateral force signs");
  assert(abs(dut.chassisFrame.f[3])<1e-8, "Contact actuator must not introduce vertical force");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-120, -80}, {100, 60}}, preserveAspectRatio = true)));
end TestContactPatchForceActuator;
