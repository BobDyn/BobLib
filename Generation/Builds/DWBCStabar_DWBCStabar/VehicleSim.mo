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
    "0 - open-loop slow ramp steer; 1 - open-loop sinusoidal steer, constant velocity; 2 - custom open-loop step steer and drive torque"
    annotation(Evaluate = false);

  // Toggle controllers
  final parameter Boolean openLoopAy = useMode == 0;
  final parameter Boolean closedLoopVelocity = useMode == 0 or useMode == 1 or useMode == 2;

  parameter Modelica.SIunits.Time steerStart = 1.0
    "Start time"
    annotation(Evaluate = false);

  // Open-loop ramp-steer parameters
  parameter SIunits.Acceleration targetAy = 18
    "Nominal target lateral acceleration used to size the open-loop ramp"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SIunits.Velocity targetVel = 15
    "Target maneuver velocity"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter SIunits.Velocity initialVel = targetVel
    "Initial velocity"
    annotation(Evaluate = false);

  parameter Real steerRatioEstimateStart = 17.5
    "Geometry-based bootstrap for handwheel-to-roadwheel ratio"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Real steerRatioEstimateDecay = 15.5
    "Gain-scheduling strength for the feedforward steer ratio"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SIunits.Time ayRampDuration = 3.0
    "Open-loop ramp duration"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SIunits.Time steadyHoldDuration = 0.1
    "Duration that the QSS plateau conditions must remain true before termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Real velGain = 200
    "Proportional gain of velocity controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Real velTi = 1
    "Time constant of velocity controller"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Real der_yawVelTol = 0.01
    "Yaw-rate derivative tolerance for ramp-steer steady-state detection";

  parameter Real handwheelRateTol = 0.01
    "Handwheel-rate derivative tolerance for open-loop QSS detection";

  parameter SIunits.Time settleTimeout = 3.0
    "Fail-fast timeout after the ramp ends if QSS is never reached";

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
  Real bodyAngularVels[3];
  Real bodyAccels[3];
  Real bodyAngles[3];

  Real speed;
  Real curvature;
  Real targetRad;
  Real targetCurvature;
  Real targetAyCmd;
  Real targetCurvatureCmd;
  Real targetRoadwheel;
  Real targetRoadwheelCmd;
  Real steerRatioEstimate;
  Real steerFeedforward;
  Real roadwheelMag;

  Real rampXi;
  Real ayRampFactor;
  Real ayErrorRaw;
  Real ayError;
  Real curvatureErrorRaw;
  Real curvatureError;

  Real radError;
  Real velError;
  Real steerSine;
  Real steerStep;

  // Standard outputs
  SIunits.Acceleration accX;
  SIunits.Acceleration accY;
  SIunits.Angle handwheelAngle;
  SIunits.Angle steerExcess;
  SIunits.Torque handwheelTorque;
  SIunits.Force Fz_FL;
  SIunits.Force Fz_FR;
  SIunits.Force Fz_RL;
  SIunits.Force Fz_RR;
  SIunits.Angle leftSteerAngle;
  SIunits.Angle rightSteerAngle;
  SIunits.Angle avgSteerAngle;
  SIunits.Angle roll;
  SIunits.Angle sideslip;
  SIunits.Velocity velX;
  SIunits.Velocity velY;
  SIunits.AngularVelocity yawVel;

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Vehicle.Vehicle_DWBCStabar_DWBCStabar vehicle(
    pVehicle = pVehicle) annotation(
    Placement(transformation(origin = {0, 20}, extent = {{-45, -50}, {45, 50}})));

  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    enforceStates = false,
    v_rel_a(start = {initialVel, 0, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {100, 90}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition(
    exact = false,
    w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Sensors.RelativeAngles sprungAngles annotation(
    Placement(transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

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

  final parameter SIunits.Length wheelbase = abs(
    pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

protected
  discrete Real t_qss_hit(start = -1);
  discrete Real t_yawVel_hit(start = -1);

  Real leftWheelVector[3];
  Real rightWheelVector[3];

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
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = vehicle.pVehicleCG,
    animation = false) annotation(
    Placement(transformation(origin = {130, 90}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression velSetpointExpression(
    y = targetVel) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression velMeasurementExpression(
    y = speed) annotation(
    Placement(transformation(origin = {-70, -10}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Continuous.LimPID speedPI(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = velGain,
    Ti = velTi,
    yMax = 5000,
    yMin = -5000,
    Ni = 0.9,
    initType = Modelica.Blocks.Types.InitPID.InitialOutput,
    y_start = 0) annotation(
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
  targetRad =
    if abs(targetAy) > 1e-6 then
      noEvent(
        (if targetAy >= 0 then 1 else -1) *
        targetVel * targetVel / abs(targetAy)
      )
    else
      1e9;

  targetCurvature = noEvent(targetAy / max(targetVel * targetVel, 1e-6));
  targetRoadwheel = noEvent(atan(wheelbase * targetCurvature));

  curvature =
    bodyAngularVels[3] / max(speed, 0.1);

  radError =
    if abs(targetAy) > 1e-6 then
      abs(curvature - targetCurvature) /
      max(abs(targetCurvature), 1e-6) *
      abs(targetRad)
    else
      0;

  // Fifth-order smootherstep ramp for target Ay.
  // With steerStart = 1.0 and ayRampDuration = 3.0,
  // the target finishes ramping at t = 4.0 s.
  rampXi =
    if useMode == 0 then
      noEvent(
        if time <= steerStart then
          0
        elseif time >= steerStart + ayRampDuration then
          1
        else
          (time - steerStart) / ayRampDuration
      )
    else
      0;

  ayRampFactor =
    if useMode == 0 then
      noEvent(10*rampXi^3 - 15*rampXi^4 + 6*rampXi^5)
    else
      0;

  targetAyCmd =
    if useMode == 0 and noEvent(time >= steerStart) then
      ayRampFactor * targetAy
    else
      0;

  targetCurvatureCmd =
    if useMode == 0 and noEvent(time >= steerStart) then
      ayRampFactor * targetCurvature
    else
      0;

  targetRoadwheelCmd =
    if useMode == 0 and noEvent(time >= steerStart) then
      ayRampFactor * targetRoadwheel
    else
      0;

  ayErrorRaw =
    if useMode == 0 and noEvent(time >= steerStart) then
      targetAy - accY
    else
      0;

  ayError =
    if useMode == 0 and noEvent(time >= steerStart) then
      targetAyCmd - accY
    else
      0;

  curvatureErrorRaw =
    if useMode == 0 and noEvent(time >= steerStart) then
      targetCurvature - curvature
    else
      0;

  curvatureError =
    if useMode == 0 and noEvent(time >= steerStart) then
      targetCurvatureCmd - curvature
    else
      0;

  roadwheelMag =
    if useMode == 0 and noEvent(time >= steerStart) then
      sqrt(targetRoadwheelCmd*targetRoadwheelCmd + 1e-6)
    else
      0;

  steerRatioEstimate =
    if useMode == 0 and noEvent(time >= steerStart) then
      steerRatioEstimateStart /
      (1 + steerRatioEstimateDecay*roadwheelMag)
    else
      steerRatioEstimateStart;

  steerFeedforward =
    if useMode == 0 and noEvent(time >= steerStart) then
      steerRatioEstimate * targetRoadwheelCmd
    else
      0;

  // Open-loop QSS detection: the vehicle has settled when yaw rate and
  // steering rate are both flat for long enough after the ramp ends.
  when useMode == 0
     and time > steerStart + ayRampDuration
     and abs(der(yawVel)) < der_yawVelTol
     and abs(der(handwheelAngle)) < handwheelRateTol
     and pre(t_qss_hit) < 0 then
    t_qss_hit = time;

  elsewhen useMode == 0
     and time > steerStart + ayRampDuration
     and (abs(der(yawVel)) >= der_yawVelTol
          or abs(der(handwheelAngle)) >= handwheelRateTol) then
    t_qss_hit = -1;
  end when;

  when useMode == 0
     and t_qss_hit > 0
     and time > t_qss_hit + steadyHoldDuration then
    terminate("Reached open-loop ramp-steer QSS plateau");
  end when;

  when useMode == 0
     and time > steerStart + ayRampDuration + settleTimeout then
    terminate("Open-loop ramp steer did not reach QSS plateau before timeout");
  end when;

  when useMode == 2
     and time > steerStart
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
    terminate("Reached open-loop step-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
  end when;

  steerSine =
    if noEvent(useMode == 1 and time > steerStart) then
      steerAmp*sin(2*pi*steerFreq*(time - steerStart))
    else
      0;

  steerStep =
    if noEvent(time > steerStart) then
      frRampSteerHeight
    else
      0;

  // Open-loop feedforward provides the nominal handwheel command.
  frSteerCmd =
    if useMode == 0 and noEvent(time >= steerStart) then
      steerFeedforward
    elseif useMode == 1 then
      steerSine
    elseif useMode == 2 then
      steerStep
    else
      0;

  driveTorqueCmd =
    if useMode == 0 or useMode == 1 or useMode == 2 then
      speedPI.y
    else
      0;

  frSteerPosition.phi_ref = frSteerCmd;
  vehicle.uPTNTorque = driveTorqueCmd;

  bodyVels =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);

  bodyAngularVels =
    Frames.angularVelocity2(cgFreeMotion.frame_b.R);

  bodyAccels =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.a_rel_a);

  bodyAngles =
    Frames.resolve2(vehicle.chassis.cgFrame.R, sprungAngles.angles);

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
  avgSteerAngle = (leftSteerAngle + rightSteerAngle) / 2;

  handwheelAngle = vehicle.steerFlange.phi;
  steerExcess = avgSteerAngle - wheelbase * curvature;

  speed = norm(bodyVels);
  velError = targetVel - speed;

  velX = bodyVels[1];
  velY = bodyVels[2];
  yawVel = bodyAngularVels[3];
  sideslip = atan(velY / velX);

  accX = bodyAccels[1];
  accY = bodyAccels[2];

  Fz_FL = vehicle.chassis.frAxleDW.leftTire.Fz;
  Fz_FR = vehicle.chassis.frAxleDW.rightTire.Fz;
  Fz_RL = vehicle.chassis.rrAxleDW.leftTire.Fz;
  Fz_RR = vehicle.chassis.rrAxleDW.rightTire.Fz;

  roll = bodyAngles[1];

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*vehicle.steerFlange.tau;

  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{120, 90}, {110, 90}}, color = {95, 95, 95}));

  connect(fixedFL.frame_b, groundFL.frame_a) annotation(
    Line(points = {{-120, 10}, {-110, 10}}, color = {95, 95, 95}));

  connect(fixedFR.frame_b, groundFR.frame_a) annotation(
    Line(points = {{120, 10}, {110, 10}}, color = {95, 95, 95}));

  connect(fixedRL.frame_b, groundRL.frame_a) annotation(
    Line(points = {{-120, -50}, {-110, -50}}, color = {95, 95, 95}));

  connect(fixedRR.frame_b, groundRR.frame_a) annotation(
    Line(points = {{120, -50}, {110, -50}}, color = {95, 95, 95}));

  connect(velSetpointExpression.y, speedPI.u_s) annotation(
    Line(points = {{-59, -30}, {-42, -30}}, color = {0, 0, 127}));

  connect(velMeasurementExpression.y, speedPI.u_m) annotation(
    Line(points = {{-59, -10}, {-42, -10}}, color = {0, 0, 127}));

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
    experiment(StartTime = 0.5, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000",
    __OpenModelica_simulationFlags(
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = ".*"));

end VehicleSim;
