within BobLibTest.Chassis.Body;

model TestFrameCompX

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  parameter MassRecord pSprung(
    m = 100,
    rCM = {0, 0, 0.25},
    inertia = [20, 0, 0; 0, 30, 0; 0, 0, 40]) "Sprung-body mass, center of mass and inertia used by the frame fixture" annotation(Dialog(group = "Calibration"));
  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Real jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed frontFixed(r = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body rearLoad(
    m = 1,
    r_CM = {0, 0, 0},
    I_11 = 0.01,
    I_22 = 0.01,
    I_33 = 0.01) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Body.FrameCompX frame(pFrame(frontReference = {1, 0, 0}, rearReference = {-1, 0, 0}, baseMass = pSprung, torsionalStiffness = 1000)) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(frontFixed.frame_b, frame.frontFrame) annotation(
    Line(points = {{-50, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(frame.rearFrame, rearLoad.frame_a) annotation(
    Line(points = {{20, 0}, {50, 0}}, color = {95, 95, 95}));

  assert(max(abs(frame.rearFrame.r_0 - {-1, 0, 0})) < 1e-9,
    "Unloaded roll compliance must preserve the frame reference separation");
  assert(abs(frame.frontFrame.f[3] - (frame.pFrame.mass.m + rearLoad.m)*world.g) < 1e-7,
    "The front support must carry the complete sprung and rear-load weight");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 60}}, preserveAspectRatio = true)));
end TestFrameCompX;
