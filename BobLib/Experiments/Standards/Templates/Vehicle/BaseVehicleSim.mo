within BobLib.Experiments.Standards.Templates.Vehicle;

partial model BaseVehicleSim

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  // Animation geometry shared by detailed MultiBody subsystems
  inner parameter SI.Length linkDiameter = 0.020 annotation(
    Dialog(tab = "Setup", group = "Animation"));
  inner parameter SI.Length jointDiameter = 0.030 annotation(
    Dialog(tab = "Setup", group = "Animation"));
  inner parameter Boolean headless = false "Run without MultiBody animation geometry" annotation(
    Evaluate = true,
    Dialog(tab = "Setup", group = "Animation"));

  // Vehicle record and initial state
  replaceable record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  parameter VehicleRecord pVehicle = VehicleRecord() annotation(
    Dialog(tab = "Setup", group = "Vehicle Definition"));
  parameter SI.Velocity initialVel = 15 "Initial velocity" annotation(
    Evaluate = false,
    Dialog(tab = "Setup", group = "Initial Conditions"));

  // Safety termination monitors
  parameter Boolean terminateOnTireLift = true "Terminate the maneuver if a tire reaches the lift threshold" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Force tireLiftTerminateLoad = 75.0 "Tire normal load threshold used for hard lift termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter Boolean terminateOnSpinout = true "Terminate the maneuver if body sideslip indicates loss of directional control" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Angle sideslipTerminate = 20*pi/180 "Absolute body sideslip threshold used for spinout termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Time spinoutHoldDuration = 0.02 "Duration the sideslip threshold must remain true before spinout termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));

  // Open-loop lateral-linearity monitor
  parameter Boolean enableLinearityTermination = true "Terminate the open-loop ramp when steering response leaves the linear region" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Linearity"));
  parameter Real linearityNonlinearityFraction = 0.20 "Fractional local lateral-gain loss that ends the ramp" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Linearity"));
  parameter SI.Acceleration linearityReferenceAy = 4.0 "Measured lateral acceleration used to latch the linear local lateral gain" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Linearity"));
  parameter SI.Acceleration linearityEvaluationAyMargin = 0.75 "Additional measured lateral acceleration beyond the reference before nonlinearity is evaluated" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Linearity"));
  parameter SI.Time linearityHoldDuration = 0.05 "Duration that the nonlinearity threshold must remain true before termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Linearity"));

  // Generic settle and timeout monitors
  parameter SI.Time steadyHoldDuration = 0.1 "Duration that the QSS plateau conditions must remain true before termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Settling"));
  parameter Real der_yawVelTol = 0.01 "Yaw-rate derivative tolerance for ramp-steer steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Settling"));
  parameter Real handwheelRateTol = 0.01 "Handwheel-rate derivative tolerance for QSS/steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Settling"));
  parameter SI.Time settleTimeout = 3.0 "Fail-fast timeout after the ramp ends if QSS is never reached" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Settling"));

  // Closed-loop steady-state lateral-acceleration monitor
  parameter SI.Acceleration steadyStateAyTolerance = 0.05 "Allowed lateral-acceleration error for closed-loop steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter Real steadyStateAyRateTolerance(unit = "m/s3") = 0.05 "Lateral-acceleration finite-difference rate tolerance for steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter SI.Velocity steadyStateSpeedTolerance = 0.10 "Allowed vehicle-speed error for closed-loop steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter Real steadyStateYawRateDerivativeTolerance(unit = "rad/s2") = 0.01 "Yaw-rate derivative tolerance for closed-loop steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter Real steadyStateSideslipRateTolerance(unit = "rad/s") = 0.005 "Sideslip-rate tolerance for closed-loop steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter Real steadyStateRollRateTolerance(unit = "rad/s") = 0.005 "Roll-rate tolerance for closed-loop steady-state detection" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));
  parameter SI.Time steadyStateSettleTimeout = 20.0 "Fail-fast timeout after the closed-loop target reaches its final value" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));

  // Controller diagnostics forwarded from StandardVCU
  SI.Angle frSteerCmd;
  SI.Angle handwheelRampCmd;
  SI.AngularVelocity handwheelRateCmd;
  Real handwheelRampDirection;
  SI.Force minTireNormalLoad;
  Real tireNormalLoadStopXi;
  Real tireNormalLoadRateXi;
  Real tireNormalLoadRateScale;
  Boolean rampEnding;
  discrete Real linearityReferenceLateralGain(start = 0, fixed = true);
  discrete Real linearitySampleAy(start = 0, fixed = true);
  discrete Real linearitySampleHandwheel(start = 0, fixed = true);
  discrete Real linearityLocalLateralGain(start = 0, fixed = true);
  Real linearityGainRatio;
  Real linearityGainLossFraction;
  SI.Angle steerSine;
  SI.Angle steerStep;
  SI.Time steadyStateAyRampDuration;
  Real steadyStateAyRampXi;
  SI.Acceleration steadyStateAyCommand;
  Real steadyStateAyError;
  discrete Real steadyStateSpeedError(start = 0, fixed = true);
  SI.Velocity steadyStateAyIntegral;
  SI.Angle steadyStateSteerCmd;
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
  SI.Angle roll;
  SI.Angle sideslip;
  SI.Velocity velX;
  SI.Velocity velY;
  SI.AngularVelocity yawVel;

  // Vehicle plant and controller subsystem selections
  replaceable BobLib.Chassis.Chassis_DWBCStabar_DWBCStabar chassis(
    headless = headless,
    initialLongitudinalVelocity = initialVel,
    pVehicle = pVehicle) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {90, -50}, extent = {{-30, -20}, {30, 20}}))) constrainedby VehicleInterfaces.Chassis.Interfaces.TwoAxleBase "Chassis subsystem";

  replaceable BobLib.EnergyStorage.BatteryPack battery(
    includeGround = true,
    Ns = pVehicle.pBattery.Ns,
    Np = pVehicle.pBattery.Np,
    SOC_start = pVehicle.pBattery.SOC_start) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90))) constrainedby VehicleInterfaces.EnergyStorage.Interfaces.Base "Energy storage subsystem";

  replaceable BobLib.Controllers.StandardVCU vcu(
    tau_max = pVehicle.pVCU.tau_max,
    w_eps = pVehicle.pVCU.w_eps,
    motorSpeedSign = pVehicle.pVCU.motorSpeedSign,
    finalDriveRatio = pVehicle.pDriveline.finalDriveRatio,
    targetVel = initialVel,
    regenTorqueLimit = pVehicle.pVCU.regenTorqueLimit,
    mechanicalBrakeTorqueLimit = pVehicle.pVCU.mechanicalBrakeTorqueLimit,
    regenBrakeBlend = pVehicle.pVCU.regenBrakeBlend) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Controller Models"),
    Placement(transformation(origin = {130, 50}, extent = {{-10, -10}, {10, 10}}))) constrainedby BobLib.Controllers.StandardVCU "Standard vehicle control unit";

  replaceable BobLib.PowerElectronics.InverterDC inverter(
    P_max_mot = pVehicle.pInverter.P_max_mot,
    P_max_reg = pVehicle.pInverter.P_max_reg,
    V_dc_max = pVehicle.pInverter.V_dc_max) "Power electronics subsystem" annotation(
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));

  replaceable BobLib.ElectricDrives.Motor motor(
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
    rotorJ = pVehicle.pMotor.rotorJ) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-79, -10.3334}, extent = {{-10, -10}, {10, 10}}, rotation = -180))) constrainedby VehicleInterfaces.ElectricDrives.Interfaces.Base "Traction motor subsystem";

  replaceable BobLib.Transmissions.FixedRatioTransmission transmission(gearRatio = pVehicle.pDriveline.finalDriveRatio) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-40, -10}, extent = {{-10, -10}, {10, 10}}))) constrainedby VehicleInterfaces.Transmissions.Interfaces.Base "Transmission subsystem";

  replaceable BobLib.Drivelines.RearFinalDriveDifferential driveline(
    finalDriveRatio = 1,
    diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
    initialOutputAngularVelocity = initialVel/pVehicle.pRrPartialWheel.R0,
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
    halfshaftRightD = pVehicle.pDriveline.halfshaftRightD) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {25, -50}, extent = {{-15, -15}, {15, 15}}))) constrainedby VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase "Driveline subsystem";

  replaceable BobLib.Chassis.Brakes.BasicVCUBrakes brakes(maxTorque = pVehicle.pVCU.mechanicalBrakeTorqueLimit) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {155, -49}, extent = {{-15, -15}, {15, 15}}))) constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase "Brakes subsystem";
  BobLib.DriverEnvironments.Internal.Driver driverEnvironment "Driver environment publishing driver intent onto the VehicleInterfaces driver bus" annotation(
    Dialog(tab = "Subsystems", group = "Controller Models"),
    Placement(transformation(origin = {40, 60}, extent = {{-20, -20}, {20, 20}})));

  // Road, atmosphere, and world conditions
  inner replaceable VehicleInterfaces.Roads.FlatRoad road annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-60, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Roads.Interfaces.Base "Road model";
  inner replaceable BobLib.Atmospheres.ConstantAtmosphere atmosphere annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-120, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base "Atmospheric model";

  inner replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1},
    driveTrainMechanics3D = false) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-170, -150}, extent = {{-10, -10}, {10, 10}}))) constrainedby Modelica.Mechanics.MultiBody.World "Global coordinate system";

  // Shared vehicle signal namespace
  VehicleInterfaces.Interfaces.ControlBus controlBus "Control bus connector" annotation(
    Placement(
      transformation(origin = {-180, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
      iconTransformation(origin = {-100, 60}, extent = {{0, 0}, {0, 0}}, rotation = 90)));
  final parameter SI.Length wheelbase = abs(pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

protected
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

  BobLib.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.pAero,
    mountOffset = pVehicle.pAero.aeroRef - chassis.chassisReferencePosition,
    headless = headless) annotation(
    Placement(transformation(origin = {-90, -100}, extent = {{30, -20}, {-30, 20}})));

equation
  frSteerCmd = vcu.frSteerCmd;
  handwheelRampCmd = vcu.handwheelRampCmd;
  handwheelRateCmd = vcu.handwheelRateCmd;
  handwheelRampDirection = vcu.handwheelRampDirection;
  minTireNormalLoad = vcu.minTireNormalLoad;
  tireNormalLoadStopXi = vcu.tireNormalLoadStopXi;
  tireNormalLoadRateXi = vcu.tireNormalLoadRateXi;
  tireNormalLoadRateScale = vcu.tireNormalLoadRateScale;
  rampEnding = vcu.rampEnding;
  steerSine = vcu.steerSine;
  steerStep = vcu.steerStep;
  steadyStateAyRampDuration = vcu.steadyStateAyRampDuration;
  steadyStateAyRampXi = vcu.steadyStateAyRampXi;
  steadyStateAyCommand = vcu.steadyStateAyCommand;
  steadyStateAyError = vcu.steadyStateAyError;
  steadyStateAyIntegral = vcu.steadyStateAyIntegral;
  steadyStateSteerCmd = vcu.steadyStateSteerCmd;
  steadyStateTargetReached = vcu.steadyStateTargetReached;
  linearityGainRatio = if linearityReferenceValid then noEvent(abs(linearityLocalLateralGain)/max(abs(linearityReferenceLateralGain), 1e-6))
    else 1;
  linearityGainLossFraction = if vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and noEvent(abs(accY) >= linearityReferenceAy + max(0, linearityEvaluationAyMargin)) then noEvent(max(0, 1 - linearityGainRatio))
    else 0;

  when sample(vcu.steerStart, vcu.linearitySlopeSamplePeriod) then
    if vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and time >= vcu.steerStart then
      if pre(linearitySampleValid) and abs(handwheelAngle - pre(linearitySampleHandwheel)) > 1e-6 then
        linearityLocalLateralGain = (accY - pre(linearitySampleAy))/(handwheelAngle - pre(linearitySampleHandwheel));
      else
        linearityLocalLateralGain = pre(linearityLocalLateralGain);
      end if;

      if not pre(linearityReferenceValid) and pre(linearitySampleValid) and abs(accY) >= linearityReferenceAy and abs(linearityLocalLateralGain) > 1e-6 then
        linearityReferenceValid = true;
        linearityReferenceLateralGain = linearityLocalLateralGain;
      else
        linearityReferenceValid = pre(linearityReferenceValid);
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
      linearityReferenceLateralGain = pre(linearityReferenceLateralGain);
    end if;

    if vcu.useMode == vcu.MODE_STEADY_STATE_AY and time >= vcu.steerStart then
      steadyStateSpeedError = vcu.targetVel - chassis.vehicleSpeed;

      if pre(steadyStateSampleValid) then
        steadyStateAyRate = (accY - pre(steadyStateSampleAy))/max(vcu.linearitySlopeSamplePeriod, 1e-6);
        steadyStateYawRateDerivative = (yawVel - pre(steadyStateSampleYawVel))/max(vcu.linearitySlopeSamplePeriod, 1e-6);
        steadyStateSideslipRate = (sideslip - pre(steadyStateSampleSideslip))/max(vcu.linearitySlopeSamplePeriod, 1e-6);
        steadyStateRollRate = (roll - pre(steadyStateSampleRoll))/max(vcu.linearitySlopeSamplePeriod, 1e-6);
        steadyStateHandwheelRate = (handwheelAngle - pre(steadyStateSampleHandwheel))/max(vcu.linearitySlopeSamplePeriod, 1e-6);
      else
        steadyStateAyRate = pre(steadyStateAyRate);
        steadyStateYawRateDerivative = pre(steadyStateYawRateDerivative);
        steadyStateSideslipRate = pre(steadyStateSideslipRate);
        steadyStateRollRate = pre(steadyStateRollRate);
        steadyStateHandwheelRate = pre(steadyStateHandwheelRate);
      end if;
      steadyStateConditionsMet = steadyStateTargetReached and abs(steadyStateAyCommand - accY) <= steadyStateAyTolerance and abs(vcu.targetVel - chassis.vehicleSpeed) <= steadyStateSpeedTolerance and abs(steadyStateAyRate) <= steadyStateAyRateTolerance and abs(steadyStateYawRateDerivative) <= steadyStateYawRateDerivativeTolerance and abs(steadyStateSideslipRate) <= steadyStateSideslipRateTolerance and abs(steadyStateRollRate) <= steadyStateRollRateTolerance and abs(steadyStateHandwheelRate) <= handwheelRateTol;
      steadyStateSampleValid = true;
      steadyStateSampleAy = accY;
      steadyStateSampleYawVel = yawVel;
      steadyStateSampleSideslip = sideslip;
      steadyStateSampleRoll = roll;
      steadyStateSampleHandwheel = handwheelAngle;
    else
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

  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and linearityGainLossFraction >= linearityNonlinearityFraction and pre(t_linearity_limit_hit) < 0 then
    t_linearity_limit_hit = time;
  elsewhen vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and linearityGainLossFraction < linearityNonlinearityFraction then
    t_linearity_limit_hit = -1;
  end when;

  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and t_linearity_limit_hit > 0 and time > t_linearity_limit_hit + linearityHoldDuration then
    terminate("Reached steering lateral-gain loss threshold");
  end when;

  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and rampEnding and not pre(rampEnding) then
    t_ramp_end_hit = time;
  end when;

  // Open-loop QSS detection starts after the normal-load floor ends the
  // handwheel ramp, then waits for yaw and steering rates to flatten.
  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and rampEnding and abs(der(yawVel)) < der_yawVelTol and abs(der(handwheelAngle)) < handwheelRateTol and pre(t_qss_hit) < 0 then
    t_qss_hit = time;
  elsewhen vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and rampEnding and (abs(der(yawVel)) >= der_yawVelTol or abs(der(handwheelAngle)) >= handwheelRateTol) then
    t_qss_hit = -1;
  end when;

  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and t_qss_hit > 0 and time > t_qss_hit + steadyHoldDuration then
    terminate("Reached open-loop ramp-steer QSS plateau");
  end when;

  when vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP and rampEnding and time > t_ramp_end_hit + vcu.handwheelRampStopDuration + settleTimeout then
    terminate("Open-loop ramp steer did not reach QSS plateau before timeout");
  end when;

  when steadyStateConditionsMet and pre(t_steady_state_hit) < 0 then
    t_steady_state_hit = time;
  elsewhen vcu.useMode == vcu.MODE_STEADY_STATE_AY and not steadyStateConditionsMet then
    t_steady_state_hit = -1;
  end when;

  when vcu.useMode == vcu.MODE_STEADY_STATE_AY and t_steady_state_hit > 0 and time > t_steady_state_hit + steadyHoldDuration then
    terminate("Reached closed-loop steady-state lateral-acceleration target");
  end when;

  when vcu.useMode == vcu.MODE_STEADY_STATE_AY and steadyStateTargetReached and time > vcu.steerStart + steadyStateAyRampDuration + steadyStateSettleTimeout then
    terminate("Closed-loop steady-state target did not settle before timeout");
  end when;

  when (vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP or vcu.useMode == vcu.MODE_STEADY_STATE_AY) and terminateOnTireLift and time > vcu.steerStart and minTireNormalLoad <= tireLiftTerminateLoad then
    terminate("Tire normal load reached lift threshold");
  end when;

  when (vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP or vcu.useMode == vcu.MODE_STEADY_STATE_AY) and terminateOnSpinout and time > vcu.steerStart and abs(sideslip) >= sideslipTerminate and pre(t_spinout_hit) < 0 then
    t_spinout_hit = time;
  elsewhen (vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP or vcu.useMode == vcu.MODE_STEADY_STATE_AY) and terminateOnSpinout and abs(sideslip) < sideslipTerminate then
    t_spinout_hit = -1;
  end when;

  when (vcu.useMode == vcu.MODE_OPEN_LOOP_RAMP or vcu.useMode == vcu.MODE_STEADY_STATE_AY) and terminateOnSpinout and t_spinout_hit > 0 and time > t_spinout_hit + spinoutHoldDuration then
    terminate("Body sideslip reached loss-of-control threshold");
  end when;

  when vcu.useMode == vcu.MODE_STEP_STEER and time > vcu.steerStart and abs(der(yawVel)) < der_yawVelTol and pre(t_yawVel_hit) < 0 then
    t_yawVel_hit = time;
  elsewhen vcu.useMode == vcu.MODE_STEP_STEER and abs(der(yawVel)) >= der_yawVelTol then
    t_yawVel_hit = -1;
  end when;

  when vcu.useMode == vcu.MODE_STEP_STEER and t_yawVel_hit > 0 and time > t_yawVel_hit + 0.1 then
    terminate("Reached open-loop step-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
  end when;
  leftSteerAngle = chassis.leftSteerAngle;
  rightSteerAngle = chassis.rightSteerAngle;
  handwheelAngle = chassis.steeringWheel.phi;
  steerExcess = chassis.avgSteerAngle - wheelbase*chassis.bodyAngularVelocity[3]/max(chassis.vehicleSpeed, 0.1);
  velX = chassis.bodyVelocity[1];
  velY = chassis.bodyVelocity[2];
  yawVel = chassis.bodyAngularVelocity[3];
  sideslip = Modelica.Math.atan2(velY, velX);
  accX = chassis.bodyAcceleration[1];
  accY = chassis.bodyAcceleration[2];
  Fz_FL = chassis.Fz_1;
  Fz_FR = chassis.Fz_2;
  Fz_RL = chassis.Fz_3;
  Fz_RR = chassis.Fz_4;

  // Read roll directly from the chassis orientation matrix to avoid Euler branch flips.
  roll = Modelica.Math.atan2(chassis.chassisFrame.R.T[2, 3], chassis.chassisFrame.R.T[3, 3]);

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*chassis.steeringWheel.tau;
  connect(controlBus, driveline.controlBus) annotation(
    Line(points = {{-180, 30}, {1, 30}, {1, -41}, {10, -41}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, transmission.controlBus) annotation(
    Line(points = {{-180, 30}, {-59, 30}, {-59, -4}, {-50, -4}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, battery.controlBus) annotation(
    Line(points = {{-180, 30}, {-155.5, 30}, {-155.5, 0}, {-155, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{-180, 30}, {131, 30}, {131, 40}, {130, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, inverter.controlBus) annotation(
    Line(points = {{-180, 30}, {-116, 30}, {-116, -20}, {-116, -20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-180, 30}, {-100.25, 30}, {-100.25, -4}, {-89, -4}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, chassis.controlBus) annotation(
    Line(points = {{-180, 30}, {49, 30}, {49, -38}, {60, -38}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, aeroModel.controlBus) annotation(
    Line(points = {{-180, 30}, {-20, 30}, {-20, -80}, {-67.5, -80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(
    Line(points = {{-180, 30}, {131, 30}, {131, -40}, {140, -40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, driverEnvironment.controlBus) annotation(
    Line(points = {{-180, 30}, {100, 30}, {100, 72}, {60, 72}}, color = {255, 204, 51}, thickness = 0.5));
  connect(vcu.steeringAngleCommand, driverEnvironment.steeringAngleCommand) annotation(
    Line(points = {{142, 58}, {150, 58}, {150, 88}, {18, 88}, {18, 76}}, color = {0, 0, 127}));
  connect(vcu.acceleratorPedalCommand, driverEnvironment.acceleratorPedalCommand) annotation(
    Line(points = {{142, 56}, {148, 56}, {148, 84}, {18, 84}, {18, 70}}, color = {0, 0, 127}));
  connect(vcu.inverterEnableCommand, controlBus.driverBus.inverterEnable) annotation(
    Line(points = {{142, 52}, {152, 52}, {152, 30}, {-180, 30}}, color = {255, 0, 255}));
  connect(battery.pin_p, inverter.p) annotation(
    Line(points = {{-143, -20}, {-143.5, -20}, {-143.5, -24}, {-120, -24}}, color = {0, 0, 255}));
  connect(inverter.n, battery.pin_n) annotation(
    Line(points = {{-120, -36}, {-155, -36}, {-155, -20}}, color = {0, 0, 255}));
  connect(transmission.drivelineFlange, driveline.transmissionFlange) annotation(
    Line(points = {{-30, -10}, {-30, -9.75}, {-10, -9.75}, {-10, -49.5}, {10, -49.5}, {10, -50}}, color = {135, 135, 135}, thickness = 0.5));
  connect(atmosphere.atmosphereBus, aeroModel.atmosphereBus) annotation(
    Line(points = {{-135, -140}, {-135, -80}, {-79, -80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(driveline.wheelHub_2, chassis.wheelHub_2) annotation(
    Line(points = {{16, -35}, {16, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(
    Line(points = {{146, -34}, {146, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(points = {{34, -35}, {34, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(points = {{164, -34}, {164, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_1, chassis.wheelHub_1) annotation(
    Line(points = {{16, -65}, {16, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_1, chassis.wheelHub_1) annotation(
    Line(points = {{146, -64}, {146, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(points = {{34, -65}, {34, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(points = {{164, -64}, {164, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(driverEnvironment.steeringWheel, chassis.steeringWheel) annotation(
    Line(points = {{60, 60}, {60, 60.5}, {90, 60.5}, {90, -30}}));
  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{-60, -100}, {50, -100}, {50, -64}, {60, -64}}, color = {95, 95, 95}));
  connect(inverter.motor_n, motor.pin_n) annotation(
    Line(points = {{-100, -36}, {-85, -36}, {-85, -20}}, color = {0, 0, 255}));
  connect(inverter.motor_p, motor.pin_p) annotation(
    Line(points = {{-100, -24}, {-73, -24}, {-73, -20}}, color = {0, 0, 255}));
  connect(motor.shaft_b, transmission.engineFlange) annotation(
    Line(points = {{-69, -10.3334}, {-51, -10.3334}}, color = {135, 135, 135}, thickness = 0.5));
  connect(vcu.brakePedalCommand, driverEnvironment.brakePedalCommand) annotation(
    Line(points = {{142, 54}, {146, 54}, {146, 80}, {18, 80}, {18, 64}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-180, -160}, {180, 120}})),
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = "time|frSteerCmd|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|steadyState.*|linearity.*|minTireNormalLoad"),
    Documentation(info = "<html>
<p>
Partial model <code>BaseVehicleSim</code> is the shared full-vehicle simulation template.
</p>
<p>
It exposes the complete replaceable vehicle stack: chassis, brakes, driveline,
transmission, electric drive, power electronics, energy storage, VCU, road,
atmosphere, aero, and world. The template owns plant bus wiring and maneuver
termination monitors. <code>Controllers.StandardVCU</code> owns the standard
autonomous maneuver commands, EV ready-to-drive state, PTN speed-control
toggles, controller gains, target velocity, electric-drive torque, regenerative
blend, and mechanical brake request. Subsystem adapters publish the
measurements and commands they own on the shared VehicleInterfaces control bus,
and subscribers choose the fields they need.
</p>
<p>
The maneuver selector supports open-loop ramp steer, sine steer, step steer,
and closed-loop steady-state lateral-acceleration mode. In
<code>vcu.useMode = 3</code>, <code>StandardVCU</code> ramps
<code>vcu.targetAy</code>, uses a handwheel-angle PI controller to drive
measured <code>accY</code> to that target, and leaves the inherited VCU speed
controller responsible for longitudinal speed. The template terminates only
after the lateral acceleration, speed, yaw rate, sideslip, roll, and
handwheel-rate settle criteria have held for
<code>steadyHoldDuration</code>.
</p>
</html>"));
end BaseVehicleSim;
