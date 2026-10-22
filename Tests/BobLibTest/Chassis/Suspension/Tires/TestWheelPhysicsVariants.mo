within BobLibTest.Chassis.Suspension.Tires;

model TestWheelPhysicsVariants

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Wheel1DOF_YRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Wheel1DOF_ZRecord;

  parameter PartialWheelRecord pWheel(
    R0 = 0.2,
    rimR0 = 0.125,
    rimWidth = 0.18,
    staticAlpha = 0,
    staticGamma = 0) "Wheel radius, rim dimensions and static alignment used by the fixture" annotation(Dialog(group = "Calibration"));

  parameter Wheel1DOF_YRecord pWheelY(wheelJ = 0.08) "Wheel rotational inertia used by the spinning-wheel variants" annotation(Dialog(group = "Calibration"));

  parameter Wheel1DOF_ZRecord pWheelZ(
    wheelC = 30000,
    wheelD = 800) "Vertical wheel stiffness and damping used by the compliant-wheel variants" annotation(Dialog(group = "Calibration"));
  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-190, -110}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassis0(r = {0, 0, 0.2}) annotation(
    Placement(transformation(origin = {-80, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Body cp0Load(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 0, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisY(r = {0, 1, 0.2}) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cpYLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 1, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {-40, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisZ(r = {0, 2, 0.2}) annotation(
    Placement(transformation(origin = {20, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Body cpZLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 2, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisYZ(r = {0, 3, 0.2}) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cpYZLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 3, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {60, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  BobLib.Chassis.Suspension.Tires.Wheels.Wheel0DOF wheel0(pWheel(geometry = pWheel)) annotation(
    Placement(transformation(origin = {-40, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheelY(pWheel(geometry = pWheel, rotation = pWheelY)) annotation(
    Placement(transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Z wheelZ(pWheel(geometry = pWheel, vertical = pWheelZ)) annotation(
    Placement(transformation(origin = {60, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.Chassis.Suspension.Tires.Wheels.Wheel2DOF_YZ wheelYZ(pWheel(geometry = pWheel, rotation = pWheelY, vertical = pWheelZ)) annotation(
    Placement(transformation(origin = {60, -20}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(chassis0.frame_b, wheel0.chassisFrame) annotation(
    Line(points = {{-70, 90}, {-67, 90}, {-67, 70}, {-50, 70}}, color = {95, 95, 95}));
  connect(wheel0.cpFrame, cp0Load.frame_a) annotation(
    Line(points = {{-40, 60}, {-40, 50}}, color = {95, 95, 95}));
  connect(chassisY.frame_b, wheelY.chassisFrame) annotation(
    Line(points = {{-70, 0}, {-67, 0}, {-67, -20}, {-50, -20}}, color = {95, 95, 95}));
  connect(wheelY.cpFrame, cpYLoad.frame_a) annotation(
    Line(points = {{-40, -30}, {-40, -40}}, color = {95, 95, 95}));
  connect(chassisZ.frame_b, wheelZ.chassisFrame) annotation(
    Line(points = {{30, 90}, {33, 90}, {33, 70}, {50, 70}}, color = {95, 95, 95}));
  connect(wheelZ.cpFrame, cpZLoad.frame_a) annotation(
    Line(points = {{60, 60}, {60, 50}}, color = {95, 95, 95}));
  connect(chassisYZ.frame_b, wheelYZ.chassisFrame) annotation(
    Line(points = {{30, 0}, {33, 0}, {33, -20}, {50, -20}}, color = {95, 95, 95}));
  connect(wheelYZ.cpFrame, cpYZLoad.frame_a) annotation(
    Line(points = {{60, -30}, {60, -40}}, color = {95, 95, 95}));

  assert(abs(cp0Load.r_0[3]) < 1e-9 and abs(cpYLoad.r_0[3]) < 1e-9,
    "Rigid-radius wheels must hold the contact frame one unloaded radius below the hub");
  assert(abs(cpZLoad.r_0[1]) < 1e-9 and abs(cpZLoad.r_0[2] - 2) < 1e-9 and
    abs(cpYZLoad.r_0[1]) < 1e-9 and abs(cpYZLoad.r_0[2] - 3) < 1e-9,
    "Vertical compliance must not introduce lateral or longitudinal contact-frame motion");
  assert(abs(cpZLoad.r_0[3] - cpYZLoad.r_0[3]) < 1e-8,
    "Unforced spin freedom must not change the vertical response of otherwise identical wheels");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-200, -120}, {200, 140}}, preserveAspectRatio = true)));
end TestWheelPhysicsVariants;
