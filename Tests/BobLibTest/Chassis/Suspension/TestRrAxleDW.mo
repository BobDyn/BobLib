within BobLibTest.Chassis.Suspension;

model TestRrAxleDW

  import Modelica.Constants.pi;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  import BobLib.Utilities.Math.Vector;

  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));
  parameter Real leftCPInit[3] = pVehicle.chassis.rearAxle.geometry.wheelCenter + Frames.resolve1(
    Frames.axesRotations(
      {1, 2, 3},
      {pVehicle.chassis.rearAxle.tire.wheel.geometry.staticGamma*pi/180, 0, pVehicle.chassis.rearAxle.tire.wheel.geometry.staticAlpha*pi/180},
      {0, 0, 0}),
    {0, 0, -pVehicle.chassis.rearAxle.tire.wheel.geometry.R0}) "Initial left contact-patch position calculated from wheel-center position and alignment" annotation(Dialog(group = "Initialization"));
  parameter Real rightCPInit[3] = Vector.mirrorXZ(leftCPInit) "Initial right contact-patch position mirrored from the left side" annotation(Dialog(group = "Initialization"));
  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Real jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner VehicleInterfaces.Roads.FlatRoad road(final animation = false, mu = 1)
    "Flat fixture plane used by tire road queries" annotation(
    Placement(transformation(origin = {-110, -110}, extent = {{-20, -10}, {20, 10}})));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-150, -110}, extent = {{-10, -10}, {10, 10}})));

  // Steer input
  Modelica.Blocks.Sources.Ramp steerRamp(duration = 1, height = 60*Modelica.Constants.pi/180, startTime = 0) annotation(
    Placement(transformation(origin = {-80, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));

  // Left jounce input
  Modelica.Blocks.Sources.Ramp leftJounceRamp(duration = 1, height = 1*0.0254, startTime = 0) annotation(
    Placement(transformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  // Right jounce input
  Modelica.Blocks.Sources.Ramp rightJounceRamp(duration = 1, height = -1*0.0254, startTime = 0) annotation(
    Placement(transformation(origin = {110, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  // Front axle
  BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar rrAxleDW(pAxle(actuation = pVehicle.chassis.rearAxle.actuation, steering = pVehicle.chassis.rearAxle.steering, stabilizer = pVehicle.chassis.rearAxle.stabilizer, tire = pVehicle.chassis.rearAxle.tire, geometry = pVehicle.chassis.rearAxle.geometry, masses = pVehicle.chassis.rearAxle.masses),
redeclare BobLib.Chassis.Suspension.Tires.BaseTire leftTire(redeclare BobLib.Chassis.Suspension.Tires.MF52.SlipModel.NoSlip slipModel),
redeclare BobLib.Chassis.Suspension.Tires.BaseTire rightTire(redeclare BobLib.Chassis.Suspension.Tires.MF52.SlipModel.NoSlip slipModel)) annotation(
    Placement(transformation(origin = {2.72478e-07, 30.4444}, extent = {{-31.5, -14}, {31.5, 14}})));

protected

  // Left jounce input
  Modelica.Mechanics.MultiBody.Parts.Fixed leftJounceRef(animation = false, r = leftCPInit) annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Mechanics.Translational.Sources.Position leftJouncePosition(exact = true, useSupport = true) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  // Right jounce input
  Modelica.Mechanics.MultiBody.Parts.Fixed rightJounceRef(animation = false, r = rightCPInit) annotation(
    Placement(transformation(origin = {30, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.Translational.Sources.Position rightJouncePosition(exact = true, useSupport = true) annotation(
    Placement(transformation(origin = {80, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  // Axle support
  Modelica.Mechanics.MultiBody.Parts.Fixed axleFixed(r = {pVehicle.chassis.rearAxle.geometry.wheelCenter[1], 0, pVehicle.chassis.rearAxle.geometry.wheelCenter[3]}) annotation(
    Placement(transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Left jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-60, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-80, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_z(
    animation = false,
    n = {0, 0, 1},
    useAxisFlange = true) annotation(
    Placement(transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Spherical left_DOF_xyz(animation = false) annotation(
    Placement(transformation(origin = {-40, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Right jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {60, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {80, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_z(
    animation = false,
    n = {0, 0, 1},
    useAxisFlange = true) annotation(
    Placement(transformation(origin = {60, -30}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Spherical right_DOF_xyz(animation = false) annotation(
    Placement(transformation(origin = {40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  BobLib.Chassis.Interfaces.ChassisBus measurements annotation(Placement(transformation(origin = {145, 40}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
  Modelica.Blocks.Routing.RealPassThrough measuredLeftLoad annotation(Placement(transformation(origin = {210, 80}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Blocks.Routing.RealPassThrough measuredRightLoad annotation(Placement(transformation(origin = {210, 30}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Blocks.Routing.RealPassThrough measuredLeftSpring annotation(Placement(transformation(origin = {210, -20}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Blocks.Routing.RealPassThrough measuredRightSpring annotation(Placement(transformation(origin = {210, -70}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

equation
  connect(measurements, rrAxleDW.axleBus) annotation(Line(points = {{145, 40}, {145, -100}, {15, -100}, {15, 10}, {24.500000272478, 10}, {24.500000272478, 16.4444}}, color = {0, 0, 127}));
  connect(measuredLeftLoad.u, measurements.leftNormalLoad) annotation(Line(points = {{193.2, 80}, {155, 80}, {155, 40}, {145, 40}}, color = {0, 0, 127}));
  assert(
    abs(measuredLeftLoad.y - rrAxleDW.leftTire.Fz) < 1e-8,
    "Axle bus leftNormalLoad must match its physical measurement");
  connect(measuredRightLoad.u, measurements.rightNormalLoad) annotation(Line(points = {{193.2, 30}, {165, 30}, {165, 40}, {145, 40}}, color = {0, 0, 127}));
  assert(
    abs(measuredRightLoad.y - rrAxleDW.rightTire.Fz) < 1e-8,
    "Axle bus rightNormalLoad must match its physical measurement");
  connect(measuredLeftSpring.u, measurements.leftSpringLength) annotation(Line(points = {{193.2, -20}, {155, -20}, {155, 40}, {145, 40}}, color = {0, 0, 127}));
  assert(
    abs(measuredLeftSpring.y - rrAxleDW.leftSpringLength) < 1e-8,
    "Axle bus leftSpringLength must match its physical measurement");
  connect(measuredRightSpring.u, measurements.rightSpringLength) annotation(Line(points = {{193.2, -70}, {145, -70}, {145, 40}}, color = {0, 0, 127}));
  assert(
    abs(measuredRightSpring.y - rrAxleDW.rightSpringLength) < 1e-8,
    "Axle bus rightSpringLength must match its physical measurement");
  connect(axleFixed.frame_b, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, 0}, {3.4e-08, 0}, {3.4e-08, 28.6944}}, color = {95, 95, 95}));
  connect(steerRamp.y, steerPosition.phi_ref) annotation(
    Line(points = {{-69, 70}, {-62, 70}}, color = {0, 0, 127}));
  connect(leftJounceRamp.y, leftJouncePosition.s_ref) annotation(
    Line(points = {{-99, 0}, {-92, 0}}, color = {0, 0, 127}));
  connect(leftJouncePosition.support, left_DOF_z.support) annotation(
    Line(points = {{-80, -10}, {-80, -14}, {-64, -14}, {-64, -24}}, color = {0, 127, 0}));
  connect(leftJouncePosition.flange, left_DOF_z.axis) annotation(
    Line(points = {{-70, 0}, {-52, 0}, {-52, -24}}, color = {0, 127, 0}));
  connect(rightJounceRamp.y, rightJouncePosition.s_ref) annotation(
    Line(points = {{99, 0}, {92, 0}}, color = {0, 0, 127}));
  connect(rightJouncePosition.support, right_DOF_z.support) annotation(
    Line(points = {{80, -10}, {80, -14}, {64, -14}, {64, -24}}, color = {0, 127, 0}));
  connect(rightJouncePosition.flange, right_DOF_z.axis) annotation(
    Line(points = {{70, 0}, {52, 0}, {52, -24}}, color = {0, 127, 0}));
  connect(leftJounceRef.frame_b, left_DOF_x.frame_a) annotation(
    Line(points = {{-40, -70}, {-50, -70}}, color = {95, 95, 95}));
  connect(left_DOF_x.frame_b, left_DOF_y.frame_a) annotation(
    Line(points = {{-70, -70}, {-73, -70}, {-73, -64}, {-80, -64}, {-80, -60}}, color = {95, 95, 95}));
  connect(left_DOF_y.frame_b, left_DOF_z.frame_a) annotation(
    Line(points = {{-80, -40}, {-80, -30}, {-70, -30}}, color = {95, 95, 95}));
  connect(left_DOF_z.frame_b, left_DOF_xyz.frame_a) annotation(
    Line(points = {{-50, -30}, {-40, -30}, {-40, -20}}, color = {95, 95, 95}));
  connect(rightJounceRef.frame_b, right_DOF_x.frame_a) annotation(
    Line(points = {{40, -70}, {50, -70}}, color = {95, 95, 95}));
  connect(right_DOF_x.frame_b, right_DOF_y.frame_a) annotation(
    Line(points = {{70, -70}, {73, -70}, {73, -64}, {80, -64}, {80, -60}}, color = {95, 95, 95}));
  connect(right_DOF_y.frame_b, right_DOF_z.frame_a) annotation(
    Line(points = {{80, -40}, {80, -30}, {70, -30}}, color = {95, 95, 95}));
  connect(right_DOF_z.frame_b, right_DOF_xyz.frame_a) annotation(
    Line(points = {{50, -30}, {40, -30}, {40, -20}}, color = {95, 95, 95}));
  connect(steerPosition.flange, rrAxleDW.steerFlange) annotation(
    Line(points = {{-40, 70}, {-37, 70}, {-37, 50.4444}, {0, 50.4444}, {3.4e-08, 50.4444}, {3.4e-08, 37.4444}}, color = {0, 0, 0}));
  connect(rrAxleDW.leftCP, left_DOF_xyz.frame_b) annotation(
    Line(points = {{-31.499999966, 19.9444}, {-39, 19.9444}, {-39, 12}, {-40, 12}, {-40, 0}}, color = {95, 95, 95}));
  connect(rrAxleDW.rightCP, right_DOF_xyz.frame_b) annotation(
    Line(points = {{31.500000034, 19.9444}, {39, 19.9444}, {39, 12}, {40, 12}, {40, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    Diagram(coordinateSystem(extent = {{-160, -120}, {260, 120}}, preserveAspectRatio = true)),
    Icon(coordinateSystem(extent = {{-120, -100}, {120, 100}})),
    __OpenModelica_simulationFlags(
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_INIT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"));
end TestRrAxleDW;
