within BobLibVehicleInterfaces.Experiments.Standards.Templates.Vehicle;

partial model BaseVehicleSim
  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  // Import vehicle records
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  replaceable record VehicleRecord =
    BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;

  parameter VehicleRecord pVehicle = VehicleRecord();

  constant Integer MODE_OPEN_LOOP_RAMP = 0;
  constant Integer MODE_OPEN_LOOP_SINE = 1;
  constant Integer MODE_STEP_STEER = 2;
  constant Integer MODE_STEADY_STATE_AY = 3;

  parameter Integer useMode = MODE_OPEN_LOOP_RAMP
    "0 - open-loop ramp steer; 1 - open-loop sinusoidal steer; 2 - step steer; 3 - closed-loop steady-state lateral acceleration"
    annotation(Evaluate = false);

  // Toggle maneuver modes
  final parameter Boolean openLoopAy = useMode == MODE_OPEN_LOOP_RAMP;
  final parameter Boolean steadyStateAy = useMode == MODE_STEADY_STATE_AY;
  final parameter Boolean ayManeuver = openLoopAy or steadyStateAy;

  parameter Modelica.Units.SI.Time steerStart = 2.0
    "Start time"
    annotation(Evaluate = false);

  // Open-loop ramp-steer parameters
  parameter SI.Acceleration targetAy = 18
    "Target lateral acceleration; open-loop ramp mode uses only its sign"
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter SI.Velocity initialVel = 15
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
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter SI.Force tireLiftTerminateLoad = 75.0
    "Tire normal load threshold used for hard lift termination"
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter Boolean terminateOnSpinout = true
    "Terminate the maneuver if body sideslip indicates loss of directional control"
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter SI.Angle sideslipTerminate = 20*pi/180
    "Absolute body sideslip threshold used for spinout termination"
    annotation(Evaluate = false, Dialog(enable = ayManeuver and terminateOnSpinout));

  parameter SI.Time spinoutHoldDuration = 0.02
    "Duration the sideslip threshold must remain true before spinout termination"
    annotation(Evaluate = false, Dialog(enable = ayManeuver and terminateOnSpinout));

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
    "Sample period for finite-difference lateral-gain and steady-state monitors"
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter SI.Time linearityHoldDuration = 0.05
    "Duration that the nonlinearity threshold must remain true before termination"
    annotation(Evaluate = false, Dialog(enable = openLoopAy));

  parameter SI.Time steadyHoldDuration = 0.1
    "Duration that the QSS plateau conditions must remain true before termination"
    annotation(Evaluate = false, Dialog(enable = ayManeuver));

  parameter Real der_yawVelTol = 0.01
    "Yaw-rate derivative tolerance for ramp-steer steady-state detection";

  parameter Real handwheelRateTol = 0.01
    "Handwheel-rate derivative tolerance for QSS/steady-state detection";

  parameter SI.Time settleTimeout = 3.0
    "Fail-fast timeout after the ramp ends if QSS is never reached";

  // Closed-loop steady-state lateral-acceleration parameters
  parameter Real steadyStateAyRampRate(unit = "m/s3") = 2.0
    "Rate used to ramp the closed-loop lateral-acceleration target"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Angle steadyStateMaxHandwheel = 120*pi/180
    "Closed-loop steady-state steering command limit"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter Real steadyStateAyGain(unit = "rad.s2/m") = 0.050
    "Closed-loop steering PI gain from lateral-acceleration error to handwheel angle"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Time steadyStateAyTi = 1.0
    "Closed-loop steering PI integral time"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Time steadyStateSteerTimeConstant = 0.15
    "First-order lag applied to the closed-loop steering command"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Acceleration steadyStateAyTolerance = 0.05
    "Allowed lateral-acceleration error for closed-loop steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter Real steadyStateAyRateTolerance(unit = "m/s3") = 0.05
    "Lateral-acceleration finite-difference rate tolerance for steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Velocity steadyStateSpeedTolerance = 0.10
    "Allowed vehicle-speed error for closed-loop steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter Real steadyStateYawRateDerivativeTolerance(unit = "rad/s2") = 0.01
    "Yaw-rate derivative tolerance for closed-loop steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter Real steadyStateSideslipRateTolerance(unit = "rad/s") = 0.005
    "Sideslip-rate tolerance for closed-loop steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter Real steadyStateRollRateTolerance(unit = "rad/s") = 0.005
    "Roll-rate tolerance for closed-loop steady-state detection"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

  parameter SI.Time steadyStateSettleTimeout = 20.0
    "Fail-fast timeout after the closed-loop target reaches its final value"
    annotation(Evaluate = false, Dialog(enable = steadyStateAy));

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
  Real steadyStateAyRampDuration;
  Real steadyStateAyRampXi;
  SI.Acceleration steadyStateAyCommand(start = 0);
  discrete Real steadyStateAyError(start = 0, fixed = true);
  discrete Real steadyStateSpeedError(start = 0, fixed = true);
  SI.Velocity steadyStateAyIntegral(start = 0, fixed = true);
  SI.Angle steadyStateSteerCmd(start = 0, fixed = true);
  Boolean steadyStateTargetReached;
  discrete Boolean steadyStateConditionsMet(start = false, fixed = true);
  discrete Real steadyStateAyRate(start = 0, fixed = true);
  discrete Real steadyStateYawRateDerivative(start = 0, fixed = true);
  discrete Real steadyStateSideslipRate(start = 0, fixed = true);
  discrete Real steadyStateRollRate(start = 0, fixed = true);
  discrete Real steadyStateHandwheelRate(start = 0, fixed = true);

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

  replaceable BobLibVehicleInterfaces.EnergyStorage.BatteryPack battery(includeGround = true, Ns = pVehicle.pBattery.Ns, Np = pVehicle.pBattery.Np, SOC_start = pVehicle.pBattery.SOC_start) annotation(
    Placement(transformation(origin = {-6, 0}, extent = {{-128, -46}, {-98, -16}})))constrainedby VehicleInterfaces.EnergyStorage.Interfaces.Base
    "Energy storage subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-128, -46}, {-98, -16}})));

  replaceable BobLibVehicleInterfaces.Controllers.VCU vcu(
    tau_max = pVehicle.pVCU.tau_max,
    w_eps = pVehicle.pVCU.w_eps,
    motorSpeedSign = pVehicle.pVCU.motorSpeedSign,
    finalDriveRatio = pVehicle.pDriveline.finalDriveRatio,
    regenTorqueLimit = pVehicle.pVCU.regenTorqueLimit,
    mechanicalBrakeTorqueLimit = pVehicle.pVCU.mechanicalBrakeTorqueLimit,
    regenBrakeBlend = pVehicle.pVCU.regenBrakeBlend)
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
      Placement(transformation(origin = {-2, 8}, extent = {{-84, -46}, {-54, -16}})));

  replaceable BobLibVehicleInterfaces.ElectricDrives.Motor motor(Vdc_max = pVehicle.pMotor.Vdc_max, rpm_max_peak = pVehicle.pMotor.rpm_max_peak, T_peak = pVehicle.pMotor.T_peak, T_cont = pVehicle.pMotor.T_cont, I_peak_2min = pVehicle.pMotor.I_peak_2min, I_cont = pVehicle.pMotor.I_cont, Kt_Nm_per_A = pVehicle.pMotor.Kt_Nm_per_A, peakTime = pVehicle.pMotor.peakTime, P_mech_peak = pVehicle.pMotor.P_mech_peak, P_cont_low = pVehicle.pMotor.P_cont_low, P_cont_high = pVehicle.pMotor.P_cont_high, eta_mot = pVehicle.pMotor.eta_mot, eta_reg = pVehicle.pMotor.eta_reg, w_eps = pVehicle.pMotor.w_eps, rotorJ = pVehicle.pMotor.rotorJ) annotation(
    Placement(transformation(origin = {-4, -16}, extent = {{-42, -44}, {-12, -14}})))constrainedby VehicleInterfaces.ElectricDrives.Interfaces.Base
    "Traction motor subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-42, -44}, {-12, -14}})));

  replaceable BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission transmission(
    gearRatio = pVehicle.pDriveline.finalDriveRatio)
    constrainedby VehicleInterfaces.Transmissions.Interfaces.BaseAutomaticTransmission
    "Transmission subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-10, -40}, {10, -10}})));

  replaceable BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
    finalDriveRatio = 1,
    diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
    initialOutputAngularVelocity = initialVel / pVehicle.pRrPartialWheel.R0,
    diff_lockedKinematics = pVehicle.pDriveline.diff_lockedKinematics,
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
      Placement(transformation(extent = {{14, -40}, {44, -10}})));

  replaceable BobLibVehicleInterfaces.Chassis.Brakes.BasicVCUBrakes brakes(
    maxTorque = pVehicle.pVCU.mechanicalBrakeTorqueLimit)
    constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase
    "Brakes subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{120, -40}, {150, -10}})));

  BobLibVehicleInterfaces.DriverEnvironments.EVDriveByWire driverEnvironment
    "Driver environment publishing driver intent onto the VehicleInterfaces driver bus" annotation(
      Placement(transformation(origin = {0, 2}, extent = {{-20, 54}, {20, 94}})));

  inner replaceable VehicleInterfaces.Roads.FlatRoad road annotation(
    Placement(transformation(origin = {40, 0}, extent = {{-120, -140}, {-80, -120}})))
    constrainedby VehicleInterfaces.Roads.Interfaces.Base
    "Road model" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-120, -140}, {-80, -120}})));

  inner replaceable BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere annotation(
    Placement(transformation(origin = {50, 0}, extent = {{-70, -140}, {-30, -120}})))
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
  VehicleInterfaces.Interfaces.ControlBus controlBus "Control bus connector"
    annotation(Placement(transformation(
      origin = {-150, 30},
      extent = {{-20, -20}, {20, 20}},
      rotation = 90),
      iconTransformation(
      extent = {{0, 0}, {0, 0}},
      rotation = 90,
      origin = {-100, 60})));

  final parameter SI.Length wheelbase = abs(
    pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

protected
  discrete Boolean rampEnding(start = false, fixed = true);
  discrete Boolean linearitySampleValid(start = false, fixed = true);
  discrete Boolean linearityReferenceValid(start = false, fixed = true);
  discrete Boolean steadyStateSampleValid(start = false, fixed = true);
  discrete Real t_qss_hit(start = -1, fixed = true);
  discrete Real t_steady_state_hit(start = -1, fixed = true);
  discrete Real t_ramp_end_hit(start = -1, fixed = true);
  discrete Real t_linearity_limit_hit(start = -1, fixed = true);
  discrete Real t_spinout_hit(start = -1, fixed = true);
  discrete Real t_yawVel_hit(start = -1, fixed = true);
  discrete Real steadyStateSampleAy(start = 0, fixed = true);
  discrete Real steadyStateSampleYawVel(start = 0, fixed = true);
  discrete Real steadyStateSampleSideslip(start = 0, fixed = true);
  discrete Real steadyStateSampleRoll(start = 0, fixed = true);
  discrete Real steadyStateSampleHandwheel(start = 0, fixed = true);

  BobLibVehicleInterfaces.Atmospheres.Interfaces.AtmosphereBus atmosphereBus
    "Shared atmosphere signal bus populated by the atmosphere subsystem" annotation(
      Placement(transformation(extent = {{-136, -126}, {-116, -106}})));

  Modelica.Blocks.Sources.RealExpression steerCommand(
    y = frSteerCmd) annotation(
      Placement(transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.IntegerConstant requestedGearCommand(
    k = 0) "Requested automatic-transmission gear" annotation(
      Placement(transformation(origin = {-154, 78}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.IntegerConstant gearboxModeCommand(
    k = VehicleInterfaces.Types.GearMode.Drive)
    "Requested automatic-transmission mode" annotation(
      Placement(transformation(origin = {-154, 72}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.IntegerConstant ignitionCommand(
    k = VehicleInterfaces.Types.IgnitionSetting.On)
    "Ignition command published to the VehicleInterfaces driver bus" annotation(
      Placement(transformation(origin = {-154, 66}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnableCommand(k = true)
    "Inverter-enable command published to the VehicleInterfaces driver bus" annotation(
      Placement(transformation(origin = {-132, 51}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.Constant driverAcceleratorPedalCommand(k = 0)
    "Inactive driver accelerator-pedal command for VCU speed-control templates" annotation(
      Placement(transformation(origin = {-154, 92}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.Constant driverBrakePedalCommand(k = 0)
    "Inactive driver brake-pedal command for VCU speed-control templates" annotation(
      Placement(transformation(origin = {-154, 86}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.Constant driverMotorTorqueCommand(k = 0)
    "Inactive direct driver motor-torque command" annotation(
      Placement(transformation(origin = {-154, 60}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.Constant driverRegenTorqueLimitCommand(k = 0)
    "Inactive direct driver regenerative-torque command" annotation(
      Placement(transformation(origin = {-154, 54}, extent = {{-5, -2}, {5, 2}})));

  BobLibVehicleInterfaces.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.pAero,
    mountOffset = pVehicle.pAero.aeroRef - chassis.chassisReferencePosition,
    headless = headless) annotation(
      Placement(transformation(origin = {81, -100}, extent = {{-21, -20}, {21, 20}})));

equation
  assert(
    useMode == MODE_OPEN_LOOP_RAMP or
    useMode == MODE_OPEN_LOOP_SINE or
    useMode == MODE_STEP_STEER or
    useMode == MODE_STEADY_STATE_AY,
    "VehicleSim.useMode must be 0 (open-loop ramp), 1 (open-loop sine), 2 (step steer), or 3 (closed-loop steady-state lateral acceleration).");

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

  steadyStateAyRampDuration =
    noEvent(abs(targetAy) / max(steadyStateAyRampRate, 1e-6));

  steadyStateAyRampXi =
    if steadyStateAy
       and noEvent(time >= steerStart) then
      noEvent(min(1, max(0, (time - steerStart) / max(steadyStateAyRampDuration, 1e-6))))
    else
      0;

  steadyStateAyCommand =
    targetAy * noEvent(
      3*steadyStateAyRampXi^2 - 2*steadyStateAyRampXi^3);

  der(steadyStateAyIntegral) = 0;
  der(steadyStateSteerCmd) = 0;

  steadyStateTargetReached =
    steadyStateAy and noEvent(steadyStateAyRampXi >= 1);

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

    if useMode == MODE_STEADY_STATE_AY and time >= steerStart then
      steadyStateAyError = steadyStateAyCommand - accY;
      steadyStateSpeedError = vcu.targetVel - speed;

      if pre(steadyStateSampleValid) then
        steadyStateAyRate =
          (accY - pre(steadyStateSampleAy)) /
          max(linearitySlopeSamplePeriod, 1e-6);
        steadyStateYawRateDerivative =
          (yawVel - pre(steadyStateSampleYawVel)) /
          max(linearitySlopeSamplePeriod, 1e-6);
        steadyStateSideslipRate =
          (sideslip - pre(steadyStateSampleSideslip)) /
          max(linearitySlopeSamplePeriod, 1e-6);
        steadyStateRollRate =
          (roll - pre(steadyStateSampleRoll)) /
          max(linearitySlopeSamplePeriod, 1e-6);
        steadyStateHandwheelRate =
          (handwheelAngle - pre(steadyStateSampleHandwheel)) /
          max(linearitySlopeSamplePeriod, 1e-6);
      else
        steadyStateAyRate = pre(steadyStateAyRate);
        steadyStateYawRateDerivative = pre(steadyStateYawRateDerivative);
        steadyStateSideslipRate = pre(steadyStateSideslipRate);
        steadyStateRollRate = pre(steadyStateRollRate);
        steadyStateHandwheelRate = pre(steadyStateHandwheelRate);
      end if;

      steadyStateConditionsMet =
        steadyStateTargetReached
        and abs(steadyStateAyCommand - accY) <= steadyStateAyTolerance
        and abs(vcu.targetVel - speed) <= steadyStateSpeedTolerance
        and abs(steadyStateAyRate) <= steadyStateAyRateTolerance
        and abs(steadyStateYawRateDerivative) <= steadyStateYawRateDerivativeTolerance
        and abs(steadyStateSideslipRate) <= steadyStateSideslipRateTolerance
        and abs(steadyStateRollRate) <= steadyStateRollRateTolerance
        and abs(steadyStateHandwheelRate) <= handwheelRateTol;

      steadyStateSampleValid = true;
      steadyStateSampleAy = accY;
      steadyStateSampleYawVel = yawVel;
      steadyStateSampleSideslip = sideslip;
      steadyStateSampleRoll = roll;
      steadyStateSampleHandwheel = handwheelAngle;
    else
      steadyStateAyError = pre(steadyStateAyError);
      steadyStateSpeedError = pre(steadyStateSpeedError);
      steadyStateConditionsMet = false;
      steadyStateAyRate = pre(steadyStateAyRate);
      steadyStateYawRateDerivative = pre(steadyStateYawRateDerivative);
      steadyStateSideslipRate = pre(steadyStateSideslipRate);
      steadyStateRollRate = pre(steadyStateRollRate);
      steadyStateHandwheelRate = pre(steadyStateHandwheelRate);
      steadyStateSampleValid = pre(steadyStateSampleValid);
      steadyStateSampleAy = pre(steadyStateSampleAy);
      steadyStateSampleYawVel = pre(steadyStateSampleYawVel);
      steadyStateSampleSideslip = pre(steadyStateSampleSideslip);
      steadyStateSampleRoll = pre(steadyStateSampleRoll);
      steadyStateSampleHandwheel = pre(steadyStateSampleHandwheel);
    end if;
  end when;

  when sample(0, linearitySlopeSamplePeriod)
       and useMode == MODE_STEADY_STATE_AY
       and time >= steerStart then
    reinit(
      steadyStateAyIntegral,
      pre(steadyStateAyIntegral) +
      (steadyStateAyCommand - accY) * max(linearitySlopeSamplePeriod, 1e-6));
    reinit(
      steadyStateSteerCmd,
      pre(steadyStateSteerCmd) +
      min(
        1,
        max(linearitySlopeSamplePeriod, 1e-6) /
        max(steadyStateSteerTimeConstant, 1e-6)) *
      (max(
        -steadyStateMaxHandwheel,
        min(
          steadyStateMaxHandwheel,
          steadyStateAyGain *
            ((steadyStateAyCommand - accY) +
             (pre(steadyStateAyIntegral) +
              (steadyStateAyCommand - accY) * max(linearitySlopeSamplePeriod, 1e-6)) /
             max(steadyStateAyTi, 1e-6)))) -
       pre(steadyStateSteerCmd)));
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

  when steadyStateConditionsMet and pre(t_steady_state_hit) < 0 then
    t_steady_state_hit = time;

  elsewhen useMode == MODE_STEADY_STATE_AY and not steadyStateConditionsMet then
    t_steady_state_hit = -1;
  end when;

  when useMode == MODE_STEADY_STATE_AY
     and t_steady_state_hit > 0
     and time > t_steady_state_hit + steadyHoldDuration then
    terminate("Reached closed-loop steady-state lateral-acceleration target");
  end when;

  when useMode == MODE_STEADY_STATE_AY
     and steadyStateTargetReached
     and time > steerStart + steadyStateAyRampDuration + steadyStateSettleTimeout then
    terminate("Closed-loop steady-state target did not settle before timeout");
  end when;

  when (useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_STEADY_STATE_AY)
     and terminateOnTireLift
     and time > steerStart
     and minTireNormalLoad <= tireLiftTerminateLoad then
    terminate("Tire normal load reached lift threshold");
  end when;

  when (useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_STEADY_STATE_AY)
     and terminateOnSpinout
     and time > steerStart
     and abs(sideslip) >= sideslipTerminate
     and pre(t_spinout_hit) < 0 then
    t_spinout_hit = time;

  elsewhen (useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_STEADY_STATE_AY)
     and terminateOnSpinout
     and abs(sideslip) < sideslipTerminate then
    t_spinout_hit = -1;
  end when;

  when (useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_STEADY_STATE_AY)
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
    elseif useMode == MODE_STEADY_STATE_AY then
      steadyStateSteerCmd
    else
      0;

  bodyVels = chassis.bodyVelocity;
  bodyAngularVels = chassis.bodyAngularVelocity;
  bodyAccels = chassis.bodyAcceleration;

  leftSteerAngle = chassis.leftSteerAngle;
  rightSteerAngle = chassis.rightSteerAngle;
  avgSteerAngle = chassis.avgSteerAngle;

  handwheelAngle = chassis.steeringWheel.phi;
  steerExcess = avgSteerAngle - wheelbase * curvature;

  speed = chassis.vehicleSpeed;

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
    points = {{57.125, -10}, {57.125, 5}, {20, 5}, {20, -10}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(Line(
    points = {{38, -10}, {38, 0}, {90.875, 0}, {90.875, -10}},
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
    points = {{38, -40}, {38, -60}, {90.875, -60}, {90.875, -50}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_1, driveline.wheelHub_1) annotation(Line(
    points = {{57.125, -50}, {57.125, -65}, {20, -65}, {20, -40}},
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
    points = {{-150, 30}, {6, 30}, {6, -16}, {14, -16}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, transmission.controlBus) annotation(Line(
    points = {{-150, 30}, {-18, 30}, {-18, -16}, {-10, -16}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, battery.controlBus) annotation(Line(
    points = {{-150, 30}, {-138, 30}, {-138, -40}, {-134, -40}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, vcu.controlBus) annotation(Line(
    points = {{-150, 30}, {-89, 30}, {-89, 36}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, inverter.controlBus) annotation(Line(
    points = {{-150, 30}, {-150, -10}, {-86, -10}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(Line(
    points = {{-150, 30}, {-46, 30}, {-46, -54}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, chassis.controlBus) annotation(Line(
    points = {{-150, 30}, {108, 30}, {108, -8}, {44.375, -8}, {44.375, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, aeroModel.controlBus) annotation(Line(
    points = {{-150, 30}, {52, 30}, {52, -84}, {65, -84}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(atmosphere.atmosphereBus, atmosphereBus) annotation(Line(
    points = {{-10, -124}, {-114, -124}, {-114, -116}, {-126, -116}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(atmosphereBus, aeroModel.atmosphereBus) annotation(Line(
    points = {{-126, -116}, {48, -116}, {48, -92}, {65, -92}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(Line(
    points = {{-150, 30}, {115, 30}, {115, -16}, {120, -16}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, driverEnvironment.controlBus) annotation(Line(
    points = {{-150, 30}, {-34, 30}, {-34, 88}, {20, 88}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(steerCommand.y, driverEnvironment.steeringAngleCommand) annotation(
    Line(points = {{41, 70}, {44, 70}, {44, 98}, {-24, 98}, {-24, 92}}, color = {0, 0, 127}));
  connect(driverAcceleratorPedalCommand.y, driverEnvironment.acceleratorPedalCommand) annotation(
    Line(points = {{-148.5, 92}, {-30, 92}, {-30, 86}, {-24, 86}}, color = {0, 0, 127}));
  connect(driverBrakePedalCommand.y, driverEnvironment.brakePedalCommand) annotation(
    Line(points = {{-148.5, 86}, {-36, 86}, {-36, 79}, {-24, 79}}, color = {0, 0, 127}));
  connect(requestedGearCommand.y, driverEnvironment.requestedGearCommand) annotation(
    Line(points = {{-148.5, 78}, {-42, 78}, {-42, 72}, {-24, 72}}, color = {255, 127, 0}));
  connect(gearboxModeCommand.y, driverEnvironment.gearboxModeCommand) annotation(
    Line(points = {{-148.5, 72}, {-48, 72}, {-48, 66}, {-24, 66}}, color = {255, 127, 0}));
  connect(ignitionCommand.y, driverEnvironment.ignitionCommand) annotation(
    Line(points = {{-148.5, 66}, {-54, 66}, {-54, 59}, {-24, 59}}, color = {255, 127, 0}));
  connect(driverMotorTorqueCommand.y, driverEnvironment.motorTorqueCommand) annotation(
    Line(points = {{-148.5, 60}, {-60, 60}, {-60, 53}, {-24, 53}}, color = {0, 0, 127}));
  connect(driverRegenTorqueLimitCommand.y, driverEnvironment.regenTorqueLimitCommand) annotation(
    Line(points = {{-148.5, 54}, {-66, 54}, {-66, 46}, {-24, 46}}, color = {0, 0, 127}));
  connect(inverterEnableCommand.y, driverEnvironment.inverterEnableCommand) annotation(
    Line(points = {{-126.5, 51}, {-72, 51}, {-72, 40}, {-24, 40}}, color = {255, 0, 255}));
  connect(driverEnvironment.steeringWheel, chassis.steeringWheel) annotation(
    Line(points = {{20, 76}, {74, 76}, {74, -10}}));
  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{60, -100}, {60, -44}, {44, -44}}, color = {95, 95, 95}));

  connect(battery.pin_p, inverter.p) annotation(
    Line(points = {{-104, -22}, {-86, -22}, {-86, -23}}, color = {0, 0, 255}));

  connect(inverter.n, battery.pin_n) annotation(
    Line(points = {{-56, -23}, {-56, -54}, {-104, -54}, {-104, -40}}, color = {0, 0, 255}));

  connect(inverter.P_out, motor.P_elec) annotation(
    Line(points = {{-71, -47}, {-61, -47}, {-61, -45}, {-49, -45}}, color = {0, 0, 127}));

  connect(motor.shaft_b, transmission.engineFlange) annotation(
    Line(points = {{-16, -45}, {-16, -36}, {-10, -36}, {-10, -25}}, color = {135, 135, 135}, thickness = 0.5));

  connect(transmission.drivelineFlange, driveline.transmissionFlange) annotation(
    Line(points = {{10, -25}, {14, -25}}, color = {135, 135, 135}, thickness = 0.5));

  annotation(
    Diagram(coordinateSystem(extent = {{-160, -150}, {175, 110}})),
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
It exposes the complete replaceable vehicle stack: chassis, brakes, driveline,
transmission, electric drive, power electronics, energy storage, VCU, road,
atmosphere, aero, and world. The template owns maneuver excitation and plant
bus wiring, while subsystem adapters publish the measurements and commands they
own on the shared VehicleInterfaces control bus. The driver environment
publishes driver intent, including zero pedal/direct EV torque commands in the
default speed-control template. The VCU owns PTN speed-control toggles,
controller gains, target velocity, electric-drive torque, regenerative blend,
and mechanical brake request. It subscribes to driver, chassis, battery, and
motor bus signals, then publishes inverter/motor requests and
<code>brakesControlBus</code> torque requests for downstream subscribers.
</p>
<p>
The maneuver selector supports open-loop ramp steer, sine steer, step steer,
and closed-loop steady-state lateral-acceleration mode. In
<code>useMode=3</code>, the template ramps <code>targetAy</code>, uses a
handwheel-angle PI controller to drive measured <code>accY</code> to that
target, leaves the VCU speed controller responsible for longitudinal speed,
and terminates only after the lateral acceleration, speed, yaw rate, sideslip,
roll, and handwheel-rate settle criteria have held for
<code>steadyHoldDuration</code>.
</p>
</html>"));

end BaseVehicleSim;
