within BobLib.Experiments.Standards.BaseClasses;

partial model PartialFourPostRig

  import Modelica.Units.SI;

  parameter BobLib.Brakes.Types.Placement frontBrakePlacement = BobLib.Brakes.Types.Placement.Outboard "Front brake location selecting upright or sprung-frame reaction paths" annotation(Dialog(group = "Brakes"));
  parameter BobLib.Brakes.Types.Placement rearBrakePlacement = BobLib.Brakes.Types.Placement.Outboard "Rear brake location selecting wheel, halfshaft or differential-input braking" annotation(Dialog(group = "Brakes"));
  parameter Real frontBrakeInputRatio(min = Modelica.Constants.small) = 1 "Front differential-input speed divided by wheel speed for the rig brake reaction" annotation(Dialog(group = "Brakes", enable = frontBrakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput));
  parameter Real rearBrakeInputRatio(min = Modelica.Constants.small) = 1 "Rear differential-input speed divided by wheel speed for the rig brake reaction" annotation(Dialog(group = "Brakes", enable = rearBrakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput));

  BobLib.Brakes.Internal.RigAxleBrake frontBrakes(useWheelFrames = true, placement = frontBrakePlacement, inputRatio = frontBrakeInputRatio,
    leftContactTorque = frAxleDW.leftContactTorque, rightContactTorque = frAxleDW.rightContactTorque) annotation(
    Placement(transformation(origin = {0, 105}, extent = {{-20, -10}, {20, 10}})));

  BobLib.Brakes.Internal.RigAxleBrake rearBrakes(useWheelFrames = true, placement = rearBrakePlacement, inputRatio = rearBrakeInputRatio,
    leftContactTorque = rrAxleDW.leftContactTorque, rightContactTorque = rrAxleDW.rightContactTorque) annotation(
    Placement(transformation(origin = {0, -20}, extent = {{-20, -10}, {20, 10}})));

  parameter SI.Angle steerMagnitude = 0 "Maximum pinion angle magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Length heaveMagnitude = 1.5*0.0254 "Maximum heave magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Angle rollMagnitude = 1.25*Modelica.Constants.pi/180 "Maximum roll magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Force forceMagnitude = 1000
    "Maximum contact patch force"
    annotation(Dialog(group = "Test Parameters"));
  inner parameter SI.Length linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter SI.Length jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(choices(checkBox = true), Evaluate = true, Dialog(tab = "Animation"));

  // Time-value tables [time, normalized value]
  final parameter Real rollTable[:, 2] = KnCInputs.rollTable "Two-column table of time and normalized roll angle command; scaled by the rig amplitude";
  final parameter Real heaveTable[:, 2] = KnCInputs.heaveTable "Two-column table of time and normalized vertical displacement command; scaled by the rig amplitude";
  final parameter Real fxTable[:, 2] = KnCInputs.fxTable "Two-column table of time and normalized longitudinal force command; scaled by the rig amplitude";
  final parameter Real fyTable[:, 2] = KnCInputs.fyTable "Two-column table of time and normalized lateral force command; scaled by the rig amplitude";

  inner VehicleInterfaces.Roads.FlatRoad road(final animation = false, mu = 1)
    "Flat fixture plane used by tire road queries" annotation(
    Placement(transformation(origin = {-130, -150}, extent = {{-20, -10}, {20, 10}})));

  inner Modelica.Mechanics.MultiBody.World world(
    final driveTrainMechanics3D = true,
    n = {0, 0, -1},
    g = 0,
    enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-170, -150}, extent = {{-10, -10}, {10, 10}})));

  replaceable BobLib.Chassis.Suspension.Interfaces.AxleBase frAxleDW constrainedby BobLib.Chassis.Suspension.Interfaces.AxleBase(
    final includeRotorEffects = false,
    final exposeWheelBearings = true) annotation(
    choicesAllMatching = true,
    Placement(transformation(origin = {0, 70}, extent = {{-36, -16}, {36, 16}})));

  replaceable BobLib.Chassis.Suspension.Interfaces.AxleBase rrAxleDW constrainedby BobLib.Chassis.Suspension.Interfaces.AxleBase(
    final steerable = false,
    final includeRotorEffects = false,
    final exposeWheelBearings = true) annotation(
    choicesAllMatching = true,
    Placement(transformation(origin = {0, -55}, extent = {{-36, -16}, {36, 16}})));

  // Front chassis actuator
  BobLib.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator frChassisActuator(axleRef = frontAxleReference) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));

  // Rear chassis actuator
  BobLib.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator rrChassisActuator(axleRef = rearAxleReference) annotation(
    Placement(transformation(origin = {0, -95}, extent = {{-10, -10}, {10, 10}})));

  // Front steer input
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) if frAxleDW.steerable annotation(
    Placement(visible = frAxleDW.steerable, transformation(origin = {-60, 115}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Ramp steerExpression(height = steerMagnitude,
                                               duration = 1,
                                               startTime = 0) if frAxleDW.steerable annotation(
    Placement(visible = frAxleDW.steerable, transformation(origin = {-110, 115}, extent = {{-10, -10}, {10, 10}})));

  // Contact patch fixtures
  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture FL_fixture(CP_init = cpInitFL) annotation(
    Placement(transformation(origin = {-65, 30}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture FR_fixture(CP_init = cpInitFR) annotation(
    Placement(transformation(origin = {65, 30}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture RL_fixture(CP_init = cpInitRL) annotation(
    Placement(transformation(origin = {-65, -95}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture RR_fixture(CP_init = cpInitRR) annotation(
    Placement(transformation(origin = {65, -95}, extent = {{-20, -20}, {20, 20}})));

  // Force actuators
  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator FL_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {-105, 58}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator FR_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {105, 58}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator RL_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {-105, -67}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator RR_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {105, -67}, extent = {{10, -10}, {-10, 10}})));

  // Heave input
  Modelica.Blocks.Sources.CombiTimeTable heaveSource(
    table = heaveTable,
    smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative2) annotation(
    Placement(transformation(origin = {-145, 75}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression heaveCommand(y = heaveMagnitude * heaveSource.y[1]) annotation(
    Placement(transformation(origin = {-133.5, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));

  // Roll input
  Modelica.Blocks.Sources.CombiTimeTable rollSource(
    table = rollTable,
    smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative2) annotation(
    Placement(transformation(origin = {-145, -20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression rollCommand(y = rollMagnitude * rollSource.y[1]) annotation(
    Placement(transformation(origin = {138.5, 90}, extent = {{10, -10}, {-10, 10}})));

  parameter SI.Position frontAxleReference[3] "Front axle mounting reference in the vehicle design coordinate system" annotation(Dialog(group = "Operating settings"));
  parameter SI.Position rearAxleReference[3] "Rear axle mounting reference in the vehicle design coordinate system" annotation(Dialog(group = "Operating settings"));
  parameter SI.Position cpInitFL[3] "Initial front-left contact-patch position in world coordinates" annotation(Dialog(group = "Operating settings"));
  parameter SI.Position cpInitFR[3] "Initial front-right contact-patch position in world coordinates" annotation(Dialog(group = "Operating settings"));
  parameter SI.Position cpInitRL[3] "Initial rear-left contact-patch position in world coordinates" annotation(Dialog(group = "Operating settings"));
  parameter SI.Position cpInitRR[3] "Initial rear-right contact-patch position in world coordinates" annotation(Dialog(group = "Operating settings"));

equation
  connect(frontBrakes.left, frAxleDW.leftTorque) annotation(Line(points = {{-20, 105}, {-44, 105}, {-44, 68}, {-36, 68}}));
  connect(
    frontBrakes.leftWheelFrame,
    frAxleDW.leftBearing) annotation(Line(visible = frontBrakes.useWheelFrames, points = {{-20, 97}, {-38, 97}, {-38, 76}, {-36, 76}}, color = {95, 95, 95}));
  connect(frontBrakes.right, frAxleDW.rightTorque) annotation(Line(points = {{20, 105}, {44, 105}, {44, 68}, {36, 68}}));
  connect(
    frontBrakes.rightWheelFrame,
    frAxleDW.rightBearing) annotation(Line(visible = frontBrakes.useWheelFrames, points = {{20, 97}, {38, 97}, {38, 76}, {36, 76}}, color = {95, 95, 95}));
  connect(
    frontBrakes.sprungFrame,
    frAxleDW.axleFrame) annotation(Line(visible = frontBrakePlacement<>BobLib.Brakes.Types.Placement.Outboard, points = {{0, 95}, {28, 95}, {28, 48}, {0, 48}, {0, 68}}, color = {95, 95, 95}));
  connect(rearBrakes.left, rrAxleDW.leftTorque) annotation(Line(points = {{-20, -20}, {-44, -20}, {-44, -57}, {-36, -57}}));
  connect(
    rearBrakes.leftWheelFrame,
    rrAxleDW.leftBearing) annotation(Line(visible = rearBrakes.useWheelFrames, points = {{-20, -28}, {-38, -28}, {-38, -49}, {-36, -49}}, color = {95, 95, 95}));
  connect(rearBrakes.right, rrAxleDW.rightTorque) annotation(Line(points = {{20, -20}, {44, -20}, {44, -57}, {36, -57}}));
  connect(
    rearBrakes.rightWheelFrame,
    rrAxleDW.rightBearing) annotation(Line(visible = rearBrakes.useWheelFrames, points = {{20, -28}, {38, -28}, {38, -49}, {36, -49}}, color = {95, 95, 95}));
  connect(
    rearBrakes.sprungFrame,
    rrAxleDW.axleFrame) annotation(Line(visible = rearBrakePlacement<>BobLib.Brakes.Types.Placement.Outboard, points = {{0, -30}, {28, -30}, {28, -77}, {0, -77}, {0, -57}}, color = {95, 95, 95}));
  connect(FL_fixture.frame_a, frAxleDW.leftCP) annotation(
    Line(points = {{-65, 38}, {-65, 58}, {-36, 58}}, color = {95, 95, 95}));
  connect(FR_fixture.frame_a, frAxleDW.rightCP) annotation(
    Line(points = {{65, 38}, {65, 58}, {36, 58}}, color = {95, 95, 95}));
  connect(steerExpression.y, steerPosition.phi_ref) annotation(
    Line(visible = frAxleDW.steerable, points = {{-99, 115}, {-72, 115}}, color = {0, 0, 127}));
  connect(frAxleDW.axleFrame, frChassisActuator.chassisFrame) annotation(
    Line(points = {{0, 68}, {0, 38}}, color = {95, 95, 95}));
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(visible = frAxleDW.steerable, points = {{-50, 115}, {-47, 115}, {-47, 82}, {0, 82}, {0, 78}}));
  connect(FL_ForceActuator.chassisFrame, frAxleDW.leftCP) annotation(
    Line(points = {{-95, 58}, {-36, 58}}, color = {95, 95, 95}));
  connect(FR_ForceActuator.chassisFrame, frAxleDW.rightCP) annotation(
    Line(points = {{95, 58}, {36, 58}}, color = {95, 95, 95}));
  connect(RL_fixture.frame_a, rrAxleDW.leftCP) annotation(
    Line(points = {{-65, -87}, {-65, -67}, {-36, -67}}, color = {95, 95, 95}));
  connect(RL_ForceActuator.chassisFrame, rrAxleDW.leftCP) annotation(
    Line(points = {{-95, -67}, {-36, -67}}, color = {95, 95, 95}));
  connect(rrChassisActuator.chassisFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -87}, {0, -57}}, color = {95, 95, 95}));
  connect(RR_fixture.frame_a, rrAxleDW.rightCP) annotation(
    Line(points = {{65, -87}, {65, -67}, {36, -67}}, color = {95, 95, 95}));
  connect(RR_ForceActuator.chassisFrame, rrAxleDW.rightCP) annotation(
    Line(points = {{95, -67}, {36, -67}}, color = {95, 95, 95}));
  connect(heaveCommand.y, frChassisActuator.heaveInput) annotation(
    Line(points = {{-122.5, 30}, {-120, 30}, {-120, 0}, {-25, 0}, {-25, 36}, {-3.6, 36}}, color = {0, 0, 127}));
  connect(heaveCommand.y, rrChassisActuator.heaveInput) annotation(
    Line(points = {{-122.5, 30}, {-120, 30}, {-120, -125}, {-25, -125}, {-25, -89}, {-3.6, -89}}, color = {0, 0, 127}));
  connect(rollCommand.y, frChassisActuator.rollInput) annotation(
    Line(points = {{127.5, 90}, {125, 90}, {125, 12}, {-12, 12}, {-12, 24}, {-3.6, 24}}, color = {0, 0, 127}));
  connect(rollCommand.y, rrChassisActuator.rollInput) annotation(
    Line(points = {{127.5, 90}, {125, 90}, {125, -113}, {-12, -113}, {-12, -101}, {-3.6, -101}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-180, -160}, {180, 140}}, preserveAspectRatio = true)),
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Partial model <code>PartialFourPostRig</code> is the shared four-post rig template.
</p>
<p>
Architecture-neutral rig mechanics. Redeclare front and rear axles and supply their reference and contact positions. Geometry-specific measurements and initialization belong to the architecture adapter. Select frontBrakePlacement and rearBrakePlacement independently. Outboard retains upright reactions, Inboard routes them to the sprung axle frame, and DifferentialInput uses one brake through the selected input ratio and an open differential. The rig evaluates static reactions for its prescribed wheel configuration; it does not simulate a friction brake stopping a rotating wheel.
</p>
</html>"));
end PartialFourPostRig;
