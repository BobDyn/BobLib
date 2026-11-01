within BobLibTest.Utilities.Mechanics.MultiBody.Actuators;
model TestContactPatchFixture

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    g = 9.81,
    enableAnimation = false) annotation(Placement(transformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body load(
    m = 1,
    r_CM = {0, 0, 0},
    I_11 = 1,
    I_22 = 1,
    I_33 = 1,
    animation = false) annotation(Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture dut(CP_init = {0, 0, 0}) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));
  Real reactionWorld[3] = Modelica.Mechanics.MultiBody.Frames.resolve1(dut.frame_a.R, dut.frame_a.f);

equation
  connect(dut.frame_a, load.frame_a) annotation(Line(points = {{0, 8}, {30, 8}, {30, 30}, {40, 30}}, color = {95, 95, 95}));
  assert(abs(dut.frame_a.r_0[3])<1e-9, "Contact fixture must constrain vertical travel");
  assert(abs(abs(reactionWorld[3])-9.81)<1e-6, "Fixture must carry body weight");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-120, -80}, {100, 80}}, preserveAspectRatio = true)));
end TestContactPatchFixture;
