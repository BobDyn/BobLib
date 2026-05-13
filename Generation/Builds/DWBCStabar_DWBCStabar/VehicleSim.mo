within BobLib.Standards;

model VehicleSim
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;

  // Import vehicle records
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;

  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;

  parameter DWBCStabar_DWBCStabarRecord pVehicle;

  parameter Integer useMode = 0
    "0 - closed-loop radius and velocity; 1 - open-loop sinusoidal steer, constant velocity; 2 - custom open-loop steer and drive torque"
    annotation(Evaluate = false);

  // Toggle controllers
  final parameter Boolean closedLoopRadius = useMode == 0;
  final parameter Boolean closedLoopVelocity = useMode == 0 or useMode == 1 or useMode == 2;

  parameter Modelica.SIunits.Time steerStart = 1.0
    "Start time"
    annotation(Evaluate = false);

  // Closed-loop parameters
  parameter SIunits.Length targetRad = 20
    "Target maneuver curvature"
    annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

  parameter SIunits.Velocity targetVel = 15
    "Target maneuver velocity"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter SIunits.Velocity initialVel = targetVel
    "Initial velocity"
    annotation(Evaluate = false);

  parameter Real curvGain = 3
    "Proportional gain of curvature controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

  parameter Real curvTi = 0.02
    "Time constant of curvature controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

  parameter Real velGain = 200
    "Proportional gain of velocity controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Real velTi = 1
    "Time constant of velocity controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Real radErrorTol = 0.002
    "SteadyStateEval radius error tolerance"
    annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

  parameter Real der_radErrorTol = 0.5
    "SteadyStateEval radius error derivative tolerance"
    annotation(Evaluate = false, Dialog(enable = closedLoopRadius));

  parameter Real der_yawVelTol = 0.01;

  // Ramp-steer parameters
  parameter SIunits.Angle frRampSteerHeight = 5*pi/180
    "Ramp steer target angle";

  parameter SIunits.Time frRampSteerDuration = 0.001
    "Ramp steer duration";

  // Frequency response parameters
  parameter SIunits.Angle steerAmp = 6*pi/180
    "Amplitude"
    annotation(Evaluate = false);

  parameter SIunits.Frequency steerFreq = 1.0
    "Frequency (Hz)"
    annotation(Evaluate = false);

  // Raw signal parameters
  Real frSteerCmd;
  Real driveTorqueCmd;
  Real bodyVels[3];
  Real bodyAccels[3];
  Real bodyAngles[3];
  Real curvature;
  Real speed;
  Real curvError;
  Real radError;
  Real velError;
  Real steerSine;
  Real steerRamp;

  // Standard outputs
  SIunits.Acceleration accX;
  SIunits.Acceleration accY;
  SIunits.Angle handwheelAngle;
  SIunits.Torque handwheelTorque;
  SIunits.Angle leftSteerAngle;
  SIunits.Angle rightSteerAngle;
  SIunits.Angle roll;
  SIunits.Angle sideslip;
  SIunits.Velocity velX;
  SIunits.Velocity velY;
  SIunits.AngularVelocity yawVel;

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));

  // Vehicle
  BobLib.Vehicle.Vehicle_DWBCStabar_DWBCStabar vehicle(
    pVehicle = pVehicle) annotation(
    Placement(transformation(origin = {0, 20}, extent = {{-45, -50}, {45, 50}})));

  // CG motion
  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    enforceStates = false,
    v_rel_a(start = {initialVel, 0, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {100, 90}, extent = {{10, -10}, {-10, 10}})));

  // Front steer position
  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  // Body attitude sensor
  Modelica.Mechanics.MultiBody.Sensors.RelativeAngles sprungAngles annotation(
    Placement(transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Calculated parameters
  final parameter Real cpInitFL[3] =
    pVehicle.pFrDW.wheelCenter +
    Frames.resolve1(
      Frames.axesRotations(
        {1, 2, 3},
        {
          pVehicle.pFrPartialWheel.staticGamma*pi/180,
          0,
          pVehicle.pFrPartialWheel.staticAlpha*pi/180
        },
        {0, 0, 0}),
      {0, 0, -pVehicle.pFrPartialWheel.R0});

  final parameter Real cpInitFR[3] = Vector.mirrorXZ(cpInitFL);

  final parameter Real cpInitRL[3] =
    pVehicle.pRrDW.wheelCenter +
    Frames.resolve1(
      Frames.axesRotations(
        {1, 2, 3},
        {
          pVehicle.pRrPartialWheel.staticGamma*pi/180,
          0,
          pVehicle.pRrPartialWheel.staticAlpha*pi/180
        },
        {0, 0, 0}),
      {0, 0, -pVehicle.pRrPartialWheel.R0});

  final parameter Real cpInitRR[3] = Vector.mirrorXZ(cpInitRL);

protected
  // QSS detection variables
  discrete Real t_curv_hit(start = -1);
  discrete Real t_yawVel_hit(start = -1);

  Real leftWheelVector[3];
  Real rightWheelVector[3];

  // Initial geometry
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFL(
    r = cpInitFL,
    animation = false) annotation(
    Placement(transformation(origin = {-130, 10}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFR(
    r = cpInitFR,
    animation = false) annotation(
    Placement(transformation(origin = {130, 10}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRL(
    r = cpInitRL,
    animation = false) annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRR(
    r = cpInitRR,
    animation = false) annotation(
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = vehicle.pVehicleCG,
    animation = false) annotation(
    Placement(transformation(origin = {130, 90}, extent = {{10, -10}, {-10, 10}})));

  // Ground interface
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));

  // Curvature controller
  Modelica.Blocks.Sources.RealExpression curvErrorExpression(
    y = curvError) annotation(
    Placement(transformation(origin = {-110, 110}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Continuous.PI curvPI(
    k = curvGain,
    T = curvTi,
    initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(
    Placement(transformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}})));

  // Speed Controller
  Modelica.Blocks.Sources.RealExpression velErrorExpression(
    y = velError) annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Continuous.PI speedPI(
    k = velGain,
    T = velTi) annotation(
    Placement(transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}})));

initial equation
  vehicle.chassis.frAxleDW.leftTire.wheelModel.hubAxis.w =
    initialVel / pVehicle.pFrPartialWheel.R0;

  vehicle.chassis.frAxleDW.rightTire.wheelModel.hubAxis.w =
    initialVel / pVehicle.pFrPartialWheel.R0;

  vehicle.chassis.rrAxleDW.leftTire.wheelModel.hubAxis.w =
    initialVel / pVehicle.pRrPartialWheel.R0;

  vehicle.chassis.rrAxleDW.rightTire.wheelModel.hubAxis.w =
    initialVel / pVehicle.pRrPartialWheel.R0;

equation
  // Curvature quantities
  curvature =
    vehicle.chassis.spaceFrame.sprungBody.w_a[3] / max(speed, 0.1);

  // SteadyStateEval-style radius error
  radError =
    abs(speed / max(abs(vehicle.chassis.spaceFrame.sprungBody.w_a[3]), 0.1)
    - abs(targetRad));

  // SteadyStateEval-style QSS detection, only active for useMode == 0
  when useMode == 0
     and abs(radError) < radErrorTol
     and abs(der(radError)) < der_radErrorTol
     and pre(t_curv_hit) < 0 then
    t_curv_hit = time;
  end when;

  when useMode == 0
     and t_curv_hit > 0
     and time > t_curv_hit + 0.1 then
    terminate("Reached steady-state (held 0.1s)");
  end when;

  // Ramp-steer steady-state detection
  when useMode == 2
     and time > steerStart + frRampSteerDuration
     and abs(der(yawVel)) < der_yawVelTol
     and pre(t_yawVel_hit) < 0 then
    t_yawVel_hit = time;

  elsewhen useMode == 2
     and abs(der(yawVel)) >= der_yawVelTol then
    t_yawVel_hit = -1;
  end when;

  when useMode == 2
     and t_yawVel_hit > 0
     and time > t_yawVel_hit + 0.1 then
    terminate("Reached ramp-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
  end when;

  // SteadyStateEval curvature controller for useMode == 0.
  // Intentionally matches old SteadyStateEval: ramp from t = 1.0 to t = 1.2.
  curvError =
    if useMode == 0 then
      smooth(1, min(1, max(0, (time - 1)/0.2))) *
      (1/targetRad - vehicle.chassis.spaceFrame.sprungBody.w_a[3]/max(speed, 0.1))
    else
      0;

  // Sinusoidal steering profile for useMode == 1
  steerSine =
    if noEvent(useMode == 1 and time > steerStart) then
      steerAmp*sin(2*pi*steerFreq*(time - steerStart))
    else
      0;

  // Ramp-steer profile for useMode == 2
  steerRamp =
    frRampSteerHeight *
    noEvent(min(1, max(0, (time - steerStart) / frRampSteerDuration)));

  // Mode switching logic
  frSteerCmd =
    if useMode == 0 then
      curvPI.y
    elseif useMode == 1 then
      steerSine
    elseif useMode == 2 then
      steerRamp
    else
      0;

  driveTorqueCmd =
    if useMode == 0 or useMode == 1 or useMode == 2 then
      speedPI.y
    else
      0;

  // Apply steer and drive torque
  frSteerPosition.phi_ref = frSteerCmd;
  vehicle.uPTNTorque = driveTorqueCmd;

  // General quantities
  bodyVels =
    Frames.resolve2(
      vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
      vehicle.chassis.spaceFrame.sprungBody.v_0);

  bodyAccels =
    Frames.resolve2(
      vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
      vehicle.chassis.spaceFrame.sprungBody.a_0);

  bodyAngles =
    Frames.resolve2(
      vehicle.chassis.spaceFrame.sprungBody.frame_a.R,
      sprungAngles.angles);

  leftWheelVector =
    Frames.resolve1(
      vehicle.chassis.frAxleFrame.R,
      Frames.resolve2(vehicle.frameFL.R, {1, 0, 0}));

  rightWheelVector =
    Frames.resolve1(
      vehicle.chassis.frAxleFrame.R,
      Frames.resolve2(vehicle.frameFR.R, {1, 0, 0}));

  leftSteerAngle = -1*atan(leftWheelVector[2] / leftWheelVector[1]);
  rightSteerAngle = -1*atan(rightWheelVector[2] / rightWheelVector[1]);

  handwheelAngle = vehicle.steerFlange.phi;

  // Speed quantities
  speed = norm(bodyVels);

  velError = targetVel - speed;

  // Kinematics
  velX = bodyVels[1];
  velY = bodyVels[2];
  yawVel = vehicle.chassis.spaceFrame.sprungBody.w_a[3];
  sideslip = atan(velY / velX);

  // Accelerations
  accX = bodyAccels[1];
  accY = bodyAccels[2];

  // Vehicle response
  roll = bodyAngles[1];

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*vehicle.steerFlange.tau;

  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{120, 90}, {110, 90}}, color = {95, 95, 95}));

  connect(velErrorExpression.y, speedPI.u) annotation(
    Line(points = {{-59, -50}, {-42, -50}}, color = {0, 0, 127}));

  connect(fixedFL.frame_b, groundFL.frame_a) annotation(
    Line(points = {{-120, 10}, {-110, 10}}, color = {95, 95, 95}));

  connect(fixedFR.frame_b, groundFR.frame_a) annotation(
    Line(points = {{120, 10}, {110, 10}}, color = {95, 95, 95}));

  connect(fixedRL.frame_b, groundRL.frame_a) annotation(
    Line(points = {{-120, -50}, {-110, -50}}, color = {95, 95, 95}));

  connect(fixedRR.frame_b, groundRR.frame_a) annotation(
    Line(points = {{120, -50}, {110, -50}}, color = {95, 95, 95}));

  connect(curvErrorExpression.y, curvPI.u) annotation(
    Line(points = {{-99, 110}, {-82, 110}}, color = {0, 0, 127}));

  connect(vehicle.frameRL, groundRL.frame_b) annotation(
    Line(points = {{-44, -22}, {-100, -22}, {-100, -40}}, color = {95, 95, 95}));

  connect(groundRR.frame_b, vehicle.frameRR) annotation(
    Line(points = {{100, -40}, {100, -22}, {46, -22}}, color = {95, 95, 95}));

  connect(vehicle.frameFL, groundFL.frame_b) annotation(
    Line(points = {{-44, 38}, {-100, 38}, {-100, 20}}, color = {95, 95, 95}));

  connect(vehicle.frameFR, groundFR.frame_b) annotation(
    Line(points = {{46, 38}, {100, 38}, {100, 20}}, color = {95, 95, 95}));

  connect(frSteerPosition.flange, vehicle.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 62}, {0, 62}}));

  connect(cgFreeMotion.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{90, 90}, {70, 90}, {70, 20}, {46, 20}}, color = {95, 95, 95}));

  connect(world.frame_b, sprungAngles.frame_a) annotation(
    Line(points = {{-120, -110}, {70, -110}, {70, -80}}, color = {95, 95, 95}));

  connect(vehicle.cgFrame, sprungAngles.frame_b) annotation(
    Line(points = {{46, 20}, {70, 20}, {70, -60}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    Icon(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", noEquidistantTimeGrid = "()", s = "dassl", variableFilter = ".*"));

end VehicleSim;
