within BobLibVehicleInterfaces.Experiments.Standards.Templates;

partial model BaseVehicleSim
  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;

  // Import vehicle records
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  replaceable record VehicleRecord =
    BobLibVehicleInterfaces.Records.VehicleDefn.DWBCStabar_DWBCStabarRecord;

  parameter VehicleRecord pVehicle = VehicleRecord();

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
  SI.Velocity windVelocityWorld[3];
  SI.Velocity windVelocityBody[3];
  SI.Velocity relativeAirVelocity[3];
  SI.Velocity relativeAirSpeed;
  SI.Density airDensity;
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

  replaceable BobLibVehicleInterfaces.Chassis.Chassis_DWBCStabar_DWBCStabar chassis(
    headless = headless,
    initialLongitudinalVelocity = initialVel,
    pVehicle = pVehicle)
    constrainedby VehicleInterfaces.Chassis.Interfaces.TwoAxleBase
    "Chassis subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{44, -50}, {104, -10}})));

  replaceable BobLibVehicleInterfaces.EnergyStorage.BatteryPack battery(
    includeGround = true,
    Ns = pVehicle.pBattery.Ns,
    Np = pVehicle.pBattery.Np,
    SOC_start = pVehicle.pBattery.SOC_start)
    constrainedby VehicleInterfaces.EnergyStorage.Interfaces.Base
    "Energy storage subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-128, -46}, {-98, -16}})));

  replaceable BobLibVehicleInterfaces.Controllers.VCU vcu(
    tau_max = pVehicle.pVCU.tau_max,
    w_eps = pVehicle.pVCU.w_eps,
    motorSpeedSign = pVehicle.pVCU.motorSpeedSign)
    constrainedby VehicleInterfaces.Controllers.Interfaces.Base
    "Vehicle control unit" annotation(
      choicesAllMatching = true,
      Dialog(group = "Controller Models"),
      Placement(transformation(extent = {{-104, 36}, {-74, 66}})));

  replaceable BobLibVehicleInterfaces.PowerElectronics.InverterDC inverter(
    P_max_mot = pVehicle.pInverter.P_max_mot,
    P_max_reg = pVehicle.pInverter.P_max_reg,
    V_dc_max = pVehicle.pInverter.V_dc_max)
    "Power electronics subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-84, -46}, {-54, -16}})));

  replaceable BobLibVehicleInterfaces.ElectricDrives.Motor motor(
    Vdc_max = pVehicle.pMotor.Vdc_max,
    rpm_max_peak = pVehicle.pMotor.rpm_max_peak,
    T_peak = pVehicle.pMotor.T_peak,
    T_cont = pVehicle.pMotor.T_cont,
    I_peak_2min = pVehicle.pMotor.I_peak_2min,
    I_cont = pVehicle.pMotor.I_cont,
    Kt_Nm_per_A = pVehicle.pMotor.Kt_Nm_per_A,
    peakTime = pVehicle.pMotor.peakTime,
    P_mech_peak = pVehicle.pMotor.P_mech_peak,
    P_cont_low = pVehicle.pMotor.P_cont_low,
    P_cont_high = pVehicle.pMotor.P_cont_high,
    eta_mot = pVehicle.pMotor.eta_mot,
    eta_reg = pVehicle.pMotor.eta_reg,
    w_eps = pVehicle.pMotor.w_eps,
    rotorJ = pVehicle.pMotor.rotorJ)
    constrainedby VehicleInterfaces.ElectricDrives.Interfaces.Base
    "Traction motor subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-42, -44}, {-12, -14}})));

  replaceable BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
    finalDriveRatio = pVehicle.pDriveline.finalDriveRatio,
    diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
    diff_use_lsd = pVehicle.pDriveline.diff_use_lsd,
    diff_driveSideTorqueSign = pVehicle.pDriveline.diff_driveSideTorqueSign,
    diff_T_preload = pVehicle.pDriveline.diff_T_preload,
    diff_lockFractionAccel = pVehicle.pDriveline.diff_lockFractionAccel,
    diff_lockFractionDecel = pVehicle.pDriveline.diff_lockFractionDecel,
    diff_T_capacity_max = pVehicle.pDriveline.diff_T_capacity_max,
    diff_clutchEffectiveRadius = pVehicle.pDriveline.diff_clutchEffectiveRadius,
    diff_kineticFrictionRatio = pVehicle.pDriveline.diff_kineticFrictionRatio,
    diff_w_transition = pVehicle.pDriveline.diff_w_transition,
    diff_c_viscous = pVehicle.pDriveline.diff_c_viscous,
    halfshaftLeftC = pVehicle.pDriveline.halfshaftLeftC,
    halfshaftLeftD = pVehicle.pDriveline.halfshaftLeftD,
    halfshaftRightC = pVehicle.pDriveline.halfshaftRightC,
    halfshaftRightD = pVehicle.pDriveline.halfshaftRightD)
    constrainedby VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase
    "Driveline subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{12, -40}, {42, -10}})));

  replaceable VehicleInterfaces.Brakes.MinimalBrakes brakes(
    maxTorque = 1500)
    constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase
    "Brakes subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{120, -40}, {150, -10}})));

  replaceable VehicleInterfaces.DriverEnvironments.DriveByWireAutomatic driverEnvironment(
    initialAccelRequest = 0,
    finalAccelRequest = 0,
    accelTime = 0.1,
    initialBrakeRequest = 0,
    finalBrakeRequest = 0,
    brakeTime = 100)
    constrainedby VehicleInterfaces.DriverEnvironments.Interfaces.BaseAutomaticTransmission
    "Driver environment" annotation(
      choicesAllMatching = true,
      Dialog(group = "Driver Models"),
      Placement(transformation(extent = {{20, 50}, {50, 80}})));

  inner replaceable VehicleInterfaces.Roads.FlatRoad road
    constrainedby VehicleInterfaces.Roads.Interfaces.Base
    "Road model" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-120, -140}, {-80, -120}})));

  inner replaceable VehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere
    constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base
    "Atmospheric model" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-70, -140}, {-30, -120}})));

  inner replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1},
    driveTrainMechanics3D = false)
    constrainedby Modelica.Mechanics.MultiBody.World
    "Global coordinate system" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-150, -140}, {-130, -120}})));

public
  VehicleInterfaces.Interfaces.ControlBus controlBus "Control bus connector"
    annotation(Placement(transformation(
      origin = {-150, 30},
      extent = {{-20, -20}, {20, 20}},
      rotation = 90),
      iconTransformation(
      extent = {{0, 0}, {0, 0}},
      rotation = 90,
      origin = {-100, 60})));

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
    Placement(transformation(origin = {132, -90}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition(
    exact = true,
    w(start = 0)) annotation(
    Placement(transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  final parameter SI.Length wheelbase = abs(
    pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

  final parameter SI.Mass pTotalMass =
    pVehicle.pSprungMass.m +
    pVehicle.pFrAxleMass.unsprungMass.m +
    pVehicle.pFrAxleMass.ucaMass.m +
    pVehicle.pFrAxleMass.lcaMass.m +
    pVehicle.pFrAxleMass.tieMass.m +
    pVehicle.pRrAxleMass.unsprungMass.m +
    pVehicle.pRrAxleMass.ucaMass.m +
    pVehicle.pRrAxleMass.lcaMass.m +
    pVehicle.pRrAxleMass.tieMass.m;

  final parameter SI.Position pVehicleCG[3] = {
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[1] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[1] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[1] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[1] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[1] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[1] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[1] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[1] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[1]
    ) / pTotalMass,
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[2] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[2] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[2] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[2] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[2] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[2] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[2] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[2] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[2]
    ) / pTotalMass,
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[3] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[3] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[3] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[3] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[3] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[3] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[3] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[3] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[3]
    ) / pTotalMass
  };

protected
  discrete Boolean rampEnding(start = false, fixed = true);
  discrete Boolean linearitySampleValid(start = false, fixed = true);
  discrete Boolean linearityReferenceValid(start = false, fixed = true);
  discrete Real t_qss_hit(start = -1, fixed = true);
  discrete Real t_ramp_end_hit(start = -1, fixed = true);
  discrete Real t_linearity_limit_hit(start = -1, fixed = true);
  discrete Real t_spinout_hit(start = -1, fixed = true);
  discrete Real t_yawVel_hit(start = -1, fixed = true);

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = pVehicleCG,
    animation = false) annotation(
    Placement(transformation(origin = {162, -90}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression velSetpointExpression(
    y = targetVel) annotation(
    Placement(transformation(origin = {-110, -66}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression velMeasurementExpression(
    y = speed) annotation(
    Placement(transformation(origin = {-110, -96}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Continuous.LimPID speedPI(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = velGain,
    Ti = velTi,
    yMax = 5000,
    yMin = -5000,
    Ni = 0.9,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = 0) annotation(
    Placement(transformation(origin = {-70, -66}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression steerCommand(
    y = frSteerCmd) annotation(
      Placement(transformation(origin = {30, 100}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression motorTorqueCmd(
    y = driveTorqueCmd / pVehicle.pDriveline.finalDriveRatio) annotation(
      Placement(transformation(origin = {-132, 57}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression regenLimitCmd(
    y = if enablePTNRegenSpeedControl then pVehicle.pVCU.regenTorqueLimit else 0) annotation(
      Placement(transformation(origin = {-132, 45}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnable(k = true) annotation(
      Placement(transformation(origin = {-132, 51}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression motorSpeed(
    y = motor.w) annotation(
      Placement(transformation(origin = {-132, 63}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression hvVoltage(
    y = inverter.V_dc) annotation(
      Placement(transformation(origin = {-132, 39}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression hvCurrent(
    y = inverter.I_dc) annotation(
      Placement(transformation(origin = {-132, 33}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroFrontRideHeight(
    y = chassis.frontRideHeight) annotation(
      Placement(transformation(origin = {36, -62}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroRearRideHeight(
    y = chassis.rearRideHeight) annotation(
      Placement(transformation(origin = {36, -70}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroRelativeAirSpeed(
    y = relativeAirSpeed) annotation(
      Placement(transformation(origin = {36, -78}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroAirDensity(
    y = airDensity) annotation(
      Placement(transformation(origin = {36, -86}, extent = {{-5, -2}, {5, 2}})));

  BobLibVehicleInterfaces.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.pAero,
    mountOffset = pVehicle.pAero.aeroRef - pVehicleCG,
    headless = headless) annotation(
      Placement(transformation(origin = {76, -82}, extent = {{-10, -10}, {10, 10}})));

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

  bodyVels =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);

  windVelocityWorld =
    atmosphere.windVelocity(r = cgFreeMotion.frame_b.r_0);

  windVelocityBody =
    Frames.resolve2(cgFreeMotion.frame_b.R, windVelocityWorld);

  relativeAirVelocity =
    bodyVels - windVelocityBody;

  relativeAirSpeed =
    norm(relativeAirVelocity);

  airDensity =
    atmosphere.density(r = cgFreeMotion.frame_b.r_0);

  bodyAngularVels =
    Frames.angularVelocity2(cgFreeMotion.frame_b.R);

  bodyAccels =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.a_rel_a);

  leftSteerAngle = chassis.leftSteerAngle;
  rightSteerAngle = chassis.rightSteerAngle;
  avgSteerAngle = chassis.avgSteerAngle;

  handwheelAngle = chassis.steeringWheel.phi;
  steerExcess = avgSteerAngle - wheelbase * curvature;

  speed = norm(bodyVels);

  velX = bodyVels[1];
  velY = bodyVels[2];
  yawVel = bodyAngularVels[3];
  sideslip = Modelica.Math.atan2(velY, velX);

  accX = bodyAccels[1];
  accY = bodyAccels[2];

  Fz_FL = chassis.Fz_1;
  Fz_FR = chassis.Fz_2;
  Fz_RL = chassis.Fz_3;
  Fz_RR = chassis.Fz_4;

  // Read roll directly from the chassis orientation matrix to avoid Euler branch flips.
  roll = Modelica.Math.atan2(chassis.chassisFrame.R.T[2, 3], chassis.chassisFrame.R.T[3, 3]);

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*chassis.steeringWheel.tau;

  connect(chassis.wheelHub_2, driveline.wheelHub_2) annotation(Line(
    points = {{57.125, -10}, {57.125, 5}, {18, 5}, {18, -10}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(Line(
    points = {{36, -10}, {36, 0}, {90.875, 0}, {90.875, -10}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_4, brakes.wheelHub_4) annotation(Line(
    points = {{90.875, -10}, {90.875, 0}, {144, 0}, {144, -10}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(Line(
    points = {{126, -10}, {126, 5}, {57.125, 5}, {57.125, -10}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(Line(
    points = {{36, -40}, {36, -60}, {90.875, -60}, {90.875, -50}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_1, driveline.wheelHub_1) annotation(Line(
    points = {{57.125, -50}, {57.125, -65}, {18, -65}, {18, -40}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_3, brakes.wheelHub_3) annotation(Line(
    points = {{90.875, -50}, {90.875, -60}, {144, -60}, {144, -40}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_1, brakes.wheelHub_1) annotation(Line(
    points = {{57.125, -50}, {57.125, -65}, {126, -65}, {126, -40}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(controlBus, driveline.controlBus) annotation(Line(
    points = {{-150, 30}, {4, 30}, {4, -16}, {12, -16}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(battery.controlBus, controlBus.batteryBus) annotation(Line(
    points = {{-128, -40}, {-138, -40}, {-138, 30}, {-150, 30}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, vcu.controlBus) annotation(Line(
    points = {{-150, 30}, {-89, 30}, {-89, 36}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(Line(
    points = {{-150, 30}, {-46, 30}, {-46, -38}, {-42, -38}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, chassis.controlBus) annotation(Line(
    points = {{-150, 30}, {40, 30}, {40, -18}, {44.375, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(Line(
    points = {{-150, 30}, {115, 30}, {115, -16}, {120, -16}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, driverEnvironment.controlBus) annotation(Line(
    points = {{-150, 30}, {80, 30}, {80, 74}, {50, 74}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(driverEnvironment.brakePedal, brakes.brakePedal) annotation(Line(
    points = {{41, 50}, {41, 44}, {135, 44}, {135, -10}},
    color = {0, 127, 0}));
  connect(driverEnvironment.steeringWheel, chassis.steeringWheel) annotation(Line(
    points = {{50, 65}, {74, 65}, {74, -10}}));
  connect(driverEnvironment.chassisFrame, chassis.chassisFrame) annotation(Line(
    points = {{47, 50}, {47, 40}, {31, 40}, {31, -44}, {44, -44}},
    color = {95, 95, 95},
    thickness = 0.5));
  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{66, -82}, {44, -82}, {44, -44}}, color = {95, 95, 95}));
  connect(aeroFrontRideHeight.y, aeroModel.frontRideHeight) annotation(
    Line(points = {{41.5, -62}, {66, -62}, {66, -70}}, color = {0, 0, 127}));
  connect(aeroRearRideHeight.y, aeroModel.rearRideHeight) annotation(
    Line(points = {{41.5, -70}, {70, -70}}, color = {0, 0, 127}));
  connect(aeroRelativeAirSpeed.y, aeroModel.relativeAirSpeed) annotation(
    Line(points = {{41.5, -78}, {74, -78}, {74, -70}}, color = {0, 0, 127}));
  connect(aeroAirDensity.y, aeroModel.airDensity) annotation(
    Line(points = {{41.5, -86}, {78, -86}, {78, -70}}, color = {0, 0, 127}));

  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{152, -90}, {142, -90}}, color = {95, 95, 95}));

  connect(velSetpointExpression.y, speedPI.u_s) annotation(
    Line(points = {{-99, -66}, {-82, -66}}, color = {0, 0, 127}));

  connect(velMeasurementExpression.y, speedPI.u_m) annotation(
    Line(points = {{-99, -96}, {-70, -96}, {-70, -78}}, color = {0, 0, 127}));

  connect(battery.pin_p, inverter.p) annotation(
    Line(points = {{-98, -22}, {-90, -22}, {-90, -31}, {-84, -31}}, color = {0, 0, 255}));

  connect(inverter.n, battery.pin_n) annotation(
    Line(points = {{-54, -31}, {-48, -31}, {-48, -54}, {-98, -54}, {-98, -40}}, color = {0, 0, 255}));

  connect(motorTorqueCmd.y, vcu.cmd_torque_motor) annotation(
    Line(points = {{-126.5, 57}, {-107, 57}}, color = {0, 0, 127}));

  connect(regenLimitCmd.y, vcu.cmd_regen_limit) annotation(
    Line(points = {{-126.5, 45}, {-107, 45}}, color = {0, 0, 127}));

  connect(inverterEnable.y, vcu.cmd_inverter_enable) annotation(
    Line(points = {{-126.5, 51}, {-107, 51}}, color = {255, 0, 255}));

  connect(motorSpeed.y, vcu.sens_motor_speed) annotation(
    Line(points = {{-126.5, 63}, {-107, 63}}, color = {0, 0, 127}));

  connect(hvVoltage.y, vcu.sens_hv_bus_voltage) annotation(
    Line(points = {{-126.5, 39}, {-107, 39}}, color = {0, 0, 127}));

  connect(hvCurrent.y, vcu.sens_hv_bus_current) annotation(
    Line(points = {{-126.5, 33}, {-107, 33}}, color = {0, 0, 127}));

  connect(vcu.P_req, inverter.P_req) annotation(
    Line(points = {{-71, 51}, {-69, 51}, {-69, -13}}, color = {0, 0, 127}));

  connect(inverter.P_out, motor.P_elec) annotation(
    Line(points = {{-69, -49}, {-50, -49}, {-50, -29}, {-45, -29}}, color = {0, 0, 127}));

  connect(motor.shaft_b, driveline.transmissionFlange) annotation(
    Line(points = {{-12, -29}, {0, -29}, {0, -25}, {12, -25}}, color = {135, 135, 135}, thickness = 0.5));

  connect(steerCommand.y, frSteerPosition.phi_ref) annotation(
    Line(points = {{41, 100}, {50, 100}, {50, 80}}, color = {0, 0, 127}));

  connect(frSteerPosition.flange, chassis.steeringWheel) annotation(
    Line(points = {{70, 80}, {74, 80}, {74, -10}}));

  connect(cgFreeMotion.frame_b, chassis.chassisFrame) annotation(
    Line(points = {{122, -90}, {44, -90}, {44, -44}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(extent = {{-160, -150}, {175, 110}})),
    Icon(coordinateSystem(extent = {{-160, -150}, {175, 110}})),
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Partial model <code>BaseVehicleSim</code> is the shared full-vehicle simulation template.
</p>
<p>
It exposes the complete replaceable vehicle stack: chassis, brakes, driveline, electric drive, power electronics, energy storage, VCU, driver environment, road, atmosphere, aero, and world. It also contains the maneuver modes and termination monitors used by front-facing VehicleSim variants.
</p>
</html>"));

end BaseVehicleSim;
