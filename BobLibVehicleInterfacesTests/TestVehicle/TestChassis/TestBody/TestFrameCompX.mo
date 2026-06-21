within BobLibVehicleInterfacesTests.TestVehicle.TestChassis.TestBody;

model TestFrameCompX

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  inner parameter Real linkDiameter = 0.020;
  inner parameter Real jointDiameter = 0.030;
  inner parameter Boolean headless = false;

  parameter MassRecord pSprung(
    m = 100,
    rCM = {0, 0, 0.25},
    inertia = [20, 0, 0; 0, 30, 0; 0, 0, 40]);

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-60, -60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed frontFixed(r = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body rearLoad(
    m = 1,
    r_CM = {0, 0, 0},
    I_11 = 0.01,
    I_22 = 0.01,
    I_33 = 0.01) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));

  BobLibVehicleInterfaces.Chassis.Body.FrameCompX frame(
    frRef = {1, 0, 0},
    rrRef = {-1, 0, 0},
    pSprung = pSprung,
    pSprungMass = pSprung,
    torsionalStiff = 1000) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  connect(frontFixed.frame_b, frame.frontFrame) annotation(
    Line(points = {{-50, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(frame.rearFrame, rearLoad.frame_a) annotation(
    Line(points = {{50, 0}, {20, 0}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestFrameCompX;
