within BobLib.Standards.Templates;

partial model BaseVehicleSim

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;

  // Import vehicle records
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean enableAnimation = false
    "Enable MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  replaceable record VehicleRecord = BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;
  replaceable model VehicleModel = BobLib.Vehicle.Vehicle_DWBCStabar_DWBCStabar;

  parameter VehicleRecord pVehicle;

  constant Integer MODE_OPEN_LOOP_RAMP = 0;
  constant Integer MODE_OPEN_LOOP_SINE = 1;
  constant Integer MODE_STEP_STEER = 2;

  parameter Integer useMode = MODE_OPEN_LOOP_RAMP
    "0 - open-loop ramp steer; 1 - open-loop sinusoidal steer; 2 - step steer"
    annotation(Evaluate = false);

  // Toggle controllers
  final parameter Boolean openLoopAy = useMode == MODE_OPEN_LOOP_RAMP;
  final parameter Boolean closedLoopVelocity = 
    useMode == MODE_OPEN_LOOP_RAMP or
    useMode == MODE_OPEN_LOOP_SINE or
    useMode == MODE_STEP_STEER;

  parameter Boolean enablePTNDriveSpeedControl = true
    "Allow the speed controller to command rear-axle drive torque"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Boolean enablePTNRegenSpeedControl = false
    "Allow the speed controller to command rear-axle regenerative braking"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter Modelica.Units.SI.Time steerStart = 2.0
    "Start time"
    annotation(Evaluate = false);

  // Open-loop ramp-steer parameters
  parameter SI.Acceleration targetAy = 18
    "Lateral-acceleration sign used to choose open-loop handwheel ramp direction"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Velocity targetVel = 15
    "Target maneuver velocity"
    annotation(Evaluate = false, Dialog(enable = closedLoopVelocity));

  parameter SI.Velocity initialVel = targetVel
    "Initial velocity"
    annotation(Evaluate = false);

  parameter SI.AngularVelocity handwheelRampRate = 0.14
    "Open-loop handwheel ramp rate"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Time handwheelRampStopDuration = 0.18
    "Duration used to smoothly roll handwheel rate to zero after the load limit"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Boolean enableNormalLoadSteerLimiter = true
    "End the open-loop handwheel ramp when any tire reaches the load floor"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Force tireNormalLoadMin = 200.0
    "Immediate tire normal-load floor where the handwheel ramp ends"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Boolean terminateOnTireLift = true
    "Terminate the maneuver if a tire reaches the lift threshold"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Force tireLiftTerminateLoad = 75.0
    "Tire normal load threshold used for hard lift termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Boolean terminateOnSpinout = true
    "Terminate the open-loop ramp if body sideslip indicates loss of directional control"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Angle sideslipTerminate = 20*pi/180
    "Absolute body sideslip threshold used for spinout termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy and terminateOnSpinout));

  parameter SI.Time spinoutHoldDuration = 0.02
    "Duration the sideslip threshold must remain true before spinout termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy and terminateOnSpinout));

  parameter Boolean enableLinearityTermination = true
    "Terminate the open-loop ramp when steering response leaves the linear region"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter Real linearityNonlinearityFraction = 0.20
    "Fractional local lateral-gain loss that ends the ramp"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Acceleration linearityReferenceAy = 4.0
    "Measured lateral acceleration used to latch the linear local lateral gain"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Acceleration linearityEvaluationAyMargin = 0.75
    "Additional measured lateral acceleration beyond the reference before nonlinearity is evaluated"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Time linearitySlopeSamplePeriod = 0.10
    "Sample period for the finite-difference local lateral-gain monitor"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Time linearityHoldDuration = 0.05
    "Duration that the nonlinearity threshold must remain true before termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Time steadyHoldDuration = 0.1
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

  parameter SI.Time settleTimeout = 3.0
    "Fail-fast timeout after the ramp ends if QSS is never reached";

  // Ramp-steer parameters
  parameter SI.Angle frRampSteerHeight = 5*pi/180
    "Ramp steer target angle";

  parameter SI.Time frRampSteerDuration = 0.001
    "Ramp steer duration";

  parameter SI.Time stepDuration = frRampSteerDuration
    "Step steer duration";

  // Frequency response parameters
  parameter SI.Angle steerAmp = 6*pi/180
    "Amplitude"
    annotation(Evaluate = false);

  parameter SI.Frequency steerFreq = 1.0
    "Frequency (Hz)"
    annotation(Evaluate = false);

  // Raw signal parameters
  Real frSteerCmd;
  Real driveTorqueCmd;
  Real bodyVels[3];
  Real bodyAngularVels[3];
  Real bodyAccels[3];

  Real speed;
  Real curvature;
  SI.Angle handwheelRampCmd(start = 0, fixed = true);
  SI.AngularVelocity handwheelRateCmd;
  Real handwheelRampDirection;
  Real minTireNormalLoad;
  Real tireNormalLoadStopXi(start = 0, fixed = true);
  Real tireNormalLoadRateXi;
  Real tireNormalLoadRateScale;
  discrete Real linearityReferenceAyMeasured(start = 0, fixed = true);
  discrete Real linearityReferenceHandwheel(start = 0, fixed = true);
  discrete Real linearityReferenceLateralGain(start = 0, fixed = true);
  discrete Real linearitySampleAy(start = 0, fixed = true);
  discrete Real linearitySampleHandwheel(start = 0, fixed = true);
  discrete Real linearityLocalLateralGain(start = 0, fixed = true);
  Real linearityGainRatio;
  Real linearityGainLossFraction;
  Real steerSine;
  Real steerStep;

  // Standard outputs
  SI.Acceleration accX;
  SI.Acceleration accY;
  SI.Angle handwheelAngle;
  SI.Angle steerExcess;
  SI.Torque handwheelTorque;
  SI.Force Fz_FL;
  SI.Force Fz_FR;
  SI.Force Fz_RL;
  SI.Force Fz_RR;
  SI.Angle leftSteerAngle;
  SI.Angle rightSteerAngle;
  SI.Angle avgSteerAngle;
  SI.Angle roll;
  SI.Angle sideslip;
  SI.Velocity velX;
  SI.Velocity velY;
  SI.AngularVelocity yawVel;

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, enableAnimation = enableAnimation) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));

  VehicleModel vehicle(
    pVehicle = pVehicle) annotation(
    Placement(transformation(origin = {0, 20}, extent = {{-45, -50}, {45, 50}})));

  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    angles_fixed = true,
    angles_start = {0, 0, 0},
    enforceStates = true,
    useQuaternions = false,
    w_rel_a_fixed = true,
    w_rel_a_start = {0, 0, 0},
    v_rel_a(start = {initialVel, 0, 0}, each fixed = true)) annotation(
    Placement(transformation(origin = {100, 90}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition(
    exact = true,
    w(start = 0)) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

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

  final parameter SI.Length wheelbase = abs(
    pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

protected
  discrete Boolean rampEnding(start = false, fixed = true);
  discrete Boolean linearitySampleValid(start = false, fixed = true);
  discrete Boolean linearityReferenceValid(start = false, fixed = true);
  discrete Real t_qss_hit(start = -1, fixed = true);
  discrete Real t_ramp_end_hit(start = -1, fixed = true);
  discrete Real t_linearity_limit_hit(start = -1, fixed = true);
  discrete Real t_spinout_hit(start = -1, fixed = true);
  discrete Real t_yawVel_hit(start = -1, fixed = true);

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
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = vehicle.pVehicleCG,
    animation = false) annotation(
    Placement(transformation(origin = {130, 90}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression velSetpointExpression(
    y = targetVel) annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression velMeasurementExpression(
    y = speed) annotation(
    Placement(transformation(origin = {-70, -80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Continuous.LimPID speedPI(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = velGain,
    Ti = velTi,
    yMax = 5000,
    yMin = -5000,
    Ni = 0.9,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
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
  assert(
    useMode == MODE_OPEN_LOOP_RAMP or
    useMode == MODE_OPEN_LOOP_SINE or
    useMode == MODE_STEP_STEER,
    "VehicleSim.useMode must be 0 (open-loop ramp), 1 (open-loop sine), or 2 (step steer).");

  curvature = 
    bodyAngularVels[3] / max(speed, 0.1);

  minTireNormalLoad = 
    noEvent(min(min(Fz_FL, Fz_FR), min(Fz_RL, Fz_RR)));

  der(tireNormalLoadStopXi) = 
    if useMode == MODE_OPEN_LOOP_RAMP
       and enableNormalLoadSteerLimiter
       and rampEnding
       and noEvent(tireNormalLoadStopXi < 1) then
      1 / max(handwheelRampStopDuration, 1e-6)
    else
      0;

  tireNormalLoadRateXi = 
    noEvent(min(1, max(0, tireNormalLoadStopXi)));

  tireNormalLoadRateScale = 
    noEvent(1 - (3*tireNormalLoadRateXi^2 - 2*tireNormalLoadRateXi^3));

  handwheelRampDirection = 
    if noEvent(targetAy >= 0) then 1 else -1;

  handwheelRateCmd = 
    if useMode == MODE_OPEN_LOOP_RAMP
       and noEvent(time >= steerStart) then
      handwheelRampDirection * handwheelRampRate * tireNormalLoadRateScale
    else
      0;

  der(handwheelRampCmd) = handwheelRateCmd;

  linearityGainRatio = 

    if linearityReferenceValid then
      noEvent(
        abs(linearityLocalLateralGain) /
        max(abs(linearityReferenceLateralGain), 1e-6)
      )
    else
      1;

  linearityGainLossFraction = 
    if useMode == MODE_OPEN_LOOP_RAMP
       and enableLinearityTermination
       and linearityReferenceValid
       and noEvent(abs(accY) >= linearityReferenceAy + max(0, linearityEvaluationAyMargin)) then
      noEvent(max(0, 1 - linearityGainRatio))
    else
      0;

  when sample(steerStart, linearitySlopeSamplePeriod) then
    if useMode == MODE_OPEN_LOOP_RAMP and enableLinearityTermination and time >= steerStart then
      if pre(linearitySampleValid)
         and abs(handwheelAngle - pre(linearitySampleHandwheel)) > 1e-6 then
        linearityLocalLateralGain = 
          (accY - pre(linearitySampleAy)) /
          (handwheelAngle - pre(linearitySampleHandwheel));
      else
        linearityLocalLateralGain = pre(linearityLocalLateralGain);
      end if;

      if not pre(linearityReferenceValid)
         and pre(linearitySampleValid)
         and abs(accY) >= linearityReferenceAy
         and abs(linearityLocalLateralGain) > 1e-6 then
        linearityReferenceValid = true;
        linearityReferenceAyMeasured = accY;
        linearityReferenceHandwheel = handwheelAngle;
        linearityReferenceLateralGain = linearityLocalLateralGain;
      else
        linearityReferenceValid = pre(linearityReferenceValid);
        linearityReferenceAyMeasured = pre(linearityReferenceAyMeasured);
        linearityReferenceHandwheel = pre(linearityReferenceHandwheel);
        linearityReferenceLateralGain = pre(linearityReferenceLateralGain);
      end if;

      linearitySampleValid = true;
      linearitySampleAy = accY;
      linearitySampleHandwheel = handwheelAngle;
    else
      linearitySampleValid = pre(linearitySampleValid);
      linearitySampleAy = pre(linearitySampleAy);
      linearitySampleHandwheel = pre(linearitySampleHandwheel);
      linearityLocalLateralGain = pre(linearityLocalLateralGain);
      linearityReferenceValid = pre(linearityReferenceValid);
      linearityReferenceAyMeasured = pre(linearityReferenceAyMeasured);
      linearityReferenceHandwheel = pre(linearityReferenceHandwheel);
      linearityReferenceLateralGain = pre(linearityReferenceLateralGain);
    end if;
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and enableLinearityTermination
     and linearityReferenceValid
     and linearityGainLossFraction >= linearityNonlinearityFraction
     and pre(t_linearity_limit_hit) < 0 then
    t_linearity_limit_hit = time;

  elsewhen useMode == MODE_OPEN_LOOP_RAMP
     and enableLinearityTermination
     and linearityReferenceValid
     and linearityGainLossFraction < linearityNonlinearityFraction then
    t_linearity_limit_hit = -1;
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and enableLinearityTermination
     and t_linearity_limit_hit > 0
     and time > t_linearity_limit_hit + linearityHoldDuration then
    terminate("Reached steering lateral-gain loss threshold");
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and time > steerStart
     and enableNormalLoadSteerLimiter
     and minTireNormalLoad <= tireNormalLoadMin
     and not pre(rampEnding) then
    rampEnding = true;
    t_ramp_end_hit = time;
  end when;

  // Open-loop QSS detection starts after the normal-load floor ends the
  // handwheel ramp, then waits for yaw and steering rates to flatten.
  when useMode == MODE_OPEN_LOOP_RAMP
     and rampEnding
     and abs(der(yawVel)) < der_yawVelTol
     and abs(der(handwheelAngle)) < handwheelRateTol
     and pre(t_qss_hit) < 0 then
    t_qss_hit = time;

  elsewhen useMode == MODE_OPEN_LOOP_RAMP
     and rampEnding
     and (abs(der(yawVel)) >= der_yawVelTol
          or abs(der(handwheelAngle)) >= handwheelRateTol) then
    t_qss_hit = -1;
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and t_qss_hit > 0
     and time > t_qss_hit + steadyHoldDuration then
    terminate("Reached open-loop ramp-steer QSS plateau");
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and rampEnding
     and time > t_ramp_end_hit + handwheelRampStopDuration + settleTimeout then
    terminate("Open-loop ramp steer did not reach QSS plateau before timeout");
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and terminateOnTireLift
     and time > steerStart
     and minTireNormalLoad <= tireLiftTerminateLoad then
    terminate("Tire normal load reached lift threshold");
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and terminateOnSpinout
     and time > steerStart
     and abs(sideslip) >= sideslipTerminate
     and pre(t_spinout_hit) < 0 then
    t_spinout_hit = time;

  elsewhen useMode == MODE_OPEN_LOOP_RAMP
     and terminateOnSpinout
     and abs(sideslip) < sideslipTerminate then
    t_spinout_hit = -1;
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP
     and terminateOnSpinout
     and t_spinout_hit > 0
     and time > t_spinout_hit + spinoutHoldDuration then
    terminate("Body sideslip reached loss-of-control threshold");
  end when;

  when useMode == MODE_STEP_STEER
     and time > steerStart
     and abs(der(yawVel)) < der_yawVelTol
     and pre(t_yawVel_hit) < 0 then
    t_yawVel_hit = time;

  elsewhen useMode == MODE_STEP_STEER
     and abs(der(yawVel)) >= der_yawVelTol then
    t_yawVel_hit = -1;
  end when;

  when useMode == MODE_STEP_STEER
     and t_yawVel_hit > 0
     and time > t_yawVel_hit + 0.1 then
    terminate("Reached open-loop step-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
  end when;

  steerSine = 

    if noEvent(useMode == MODE_OPEN_LOOP_SINE and time > steerStart) then
      steerAmp*sin(2*pi*steerFreq*(time - steerStart))
    else
      0;

  steerStep = 

    if noEvent(time > steerStart) then
      frRampSteerHeight * noEvent(min(1, max(0, (time - steerStart) / stepDuration)))
    else
      0;

  // Open-loop mode uses a constant handwheel rate until any tire reaches
  // tireNormalLoadMin. After that event, the rate smoothly rolls to zero.
  frSteerCmd = 

    if useMode == MODE_OPEN_LOOP_RAMP and noEvent(time >= steerStart) then
      handwheelRampCmd
    elseif useMode == MODE_OPEN_LOOP_SINE then
      steerSine
    elseif useMode == MODE_STEP_STEER then
      steerStep
    else
      0;

  driveTorqueCmd = 
    if useMode == MODE_OPEN_LOOP_RAMP or
       useMode == MODE_OPEN_LOOP_SINE or
       useMode == MODE_STEP_STEER then

      if enablePTNDriveSpeedControl and enablePTNRegenSpeedControl then
        speedPI.y
      elseif enablePTNDriveSpeedControl then
        noEvent(max(speedPI.y, 0))
      elseif enablePTNRegenSpeedControl then
        noEvent(min(speedPI.y, 0))
      else
        0
    else
      0;

  frSteerPosition.phi_ref = frSteerCmd;
  vehicle.uPTNTorque = driveTorqueCmd;
  vehicle.uPTNRegenLimit = 

    if enablePTNRegenSpeedControl then
      pVehicle.pPowertrain.regenTorqueLimit
    else
      0;

  bodyVels = 
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);

  bodyAngularVels = 
    Frames.angularVelocity2(cgFreeMotion.frame_b.R);

  bodyAccels = 
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.a_rel_a);

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

  velX = bodyVels[1];
  velY = bodyVels[2];
  yawVel = bodyAngularVels[3];
  sideslip = Modelica.Math.atan2(velY, velX);

  accX = bodyAccels[1];
  accY = bodyAccels[2];

  Fz_FL = vehicle.chassis.frAxleDW.leftTire.Fz;
  Fz_FR = vehicle.chassis.frAxleDW.rightTire.Fz;
  Fz_RL = vehicle.chassis.rrAxleDW.leftTire.Fz;
  Fz_RR = vehicle.chassis.rrAxleDW.rightTire.Fz;

  // Read roll directly from the chassis orientation matrix to avoid Euler branch flips.
  roll = Modelica.Math.atan2(vehicle.chassis.cgFrame.R.T[2, 3], vehicle.chassis.cgFrame.R.T[3, 3]);

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
    Line(points = {{-59, -50}, {-42, -50}}, color = {0, 0, 127}));

  connect(velMeasurementExpression.y, speedPI.u_m) annotation(
    Line(points = {{-59, -80}, {-30, -80}, {-30, -62}}, color = {0, 0, 127}));

  connect(vehicle.frameRL, groundRL.frame_b) annotation(
    Line(points = {{-44, -22}, {-100, -22}, {-100, -40}}, color = {95, 95, 95}));

  connect(groundRR.frame_b, vehicle.frameRR) annotation(
    Line(points = {{100, -40}, {100, -22}, {46, -22}}, color = {95, 95, 95}));

  connect(vehicle.frameFL, groundFL.frame_b) annotation(
    Line(points = {{-44, 38}, {-100, 38}, {-100, 20}}, color = {95, 95, 95}));

  connect(vehicle.frameFR, groundFR.frame_b) annotation(
    Line(points = {{46, 38}, {100, 38}, {100, 20}}, color = {95, 95, 95}));

  connect(frSteerPosition.flange, vehicle.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 62}}));

  connect(cgFreeMotion.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{90, 90}, {70, 90}, {70, 20}, {46, 20}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    Icon(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = ".*"));

end BaseVehicleSim;
