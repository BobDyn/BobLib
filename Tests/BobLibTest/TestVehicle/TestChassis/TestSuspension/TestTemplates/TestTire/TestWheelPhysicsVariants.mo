within BobLibTest.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestTire;

model TestWheelPhysicsVariants

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Wheel1DOF_YRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Wheel1DOF_ZRecord;

  inner parameter Real linkDiameter = 0.020;
  inner parameter Boolean headless = false;

  parameter PartialWheelRecord pWheel(
    R0 = 0.2,
    rimR0 = 0.125,
    rimWidth = 0.18,
    staticAlpha = 0,
    staticGamma = 0);

  parameter Wheel1DOF_YRecord pWheelY(wheelJ = 0.08);

  parameter Wheel1DOF_ZRecord pWheelZ(wheelC = 30000, wheelD = 800);

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassis0(r = {0, 0, 0.2}) annotation(
    Placement(transformation(origin = {-80, 60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cp0Load(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 0, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {-80, 20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisY(r = {0, 1, 0.2}) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cpYLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 1, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {-80, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisZ(r = {0, 2, 0.2}) annotation(
    Placement(transformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cpZLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 2, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {20, 20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassisYZ(r = {0, 3, 0.2}) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body cpYZLoad(
    m = 0.1,
    r_CM = {0, 0, 0},
    I_11 = 1e-4,
    I_22 = 1e-4,
    I_33 = 1e-4,
    r_0(start = {0, 3, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.TirePhysics.Wheel0DOF wheel0(
    partialWheelParams = pWheel) annotation(
    Placement(transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.TirePhysics.Wheel1DOF_Y wheelY(
    partialWheelParams = pWheel,
    wheel1DOF_YParams = pWheelY) annotation(
    Placement(transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.TirePhysics.Wheel1DOF_Z wheelZ(
    partialWheelParams = pWheel,
    wheel1DOF_ZParams = pWheelZ) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Chassis.Suspension.Tires.TirePhysics.Wheel2DOF_YZ wheelYZ(
    partialWheelParams = pWheel,
    wheel1DOF_YParams = pWheelY,
    wheel1DOF_ZParams = pWheelZ) annotation(
    Placement(transformation(origin = {60, -20}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(chassis0.frame_b, wheel0.chassisFrame) annotation(
    Line(points = {{-70, 60}, {-50, 40}}, color = {95, 95, 95}));
  connect(wheel0.cpFrame, cp0Load.frame_a) annotation(
    Line(points = {{-70, 20}, {-40, 30}}, color = {95, 95, 95}));
  connect(chassisY.frame_b, wheelY.chassisFrame) annotation(
    Line(points = {{-70, 0}, {-50, -20}}, color = {95, 95, 95}));
  connect(wheelY.cpFrame, cpYLoad.frame_a) annotation(
    Line(points = {{-70, -40}, {-40, -30}}, color = {95, 95, 95}));
  connect(chassisZ.frame_b, wheelZ.chassisFrame) annotation(
    Line(points = {{30, 60}, {50, 40}}, color = {95, 95, 95}));
  connect(wheelZ.cpFrame, cpZLoad.frame_a) annotation(
    Line(points = {{30, 20}, {60, 30}}, color = {95, 95, 95}));
  connect(chassisYZ.frame_b, wheelYZ.chassisFrame) annotation(
    Line(points = {{30, 0}, {50, -20}}, color = {95, 95, 95}));
  connect(wheelYZ.cpFrame, cpYZLoad.frame_a) annotation(
    Line(points = {{30, -40}, {60, -30}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestWheelPhysicsVariants;
