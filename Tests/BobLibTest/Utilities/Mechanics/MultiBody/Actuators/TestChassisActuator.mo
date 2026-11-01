within BobLibTest.Utilities.Mechanics.MultiBody.Actuators;
model TestChassisActuator

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
  inner parameter Boolean headless = true "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator dut(
    axleRef = {0, 0, 1},
    heaveInput = 0.01*time,
    rollInput = 0) annotation(Placement(transformation(origin = {0, 0}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(dut.chassisFrame, load.frame_a) annotation(Line(points = {{0, 16}, {30, 16}, {30, 30}, {40, 30}}, color = {95, 95, 95}));
  assert(abs(dut.chassisFrame.r_0[3]-1-0.01*time)<1e-9, "Prescribed heave must move the axle frame");
  assert(abs(abs(dut.jackingOutput)-9.81)<1e-6, "Constant-speed heave must support body weight");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-120, -80}, {100, 80}}, preserveAspectRatio = true)));
end TestChassisActuator;
