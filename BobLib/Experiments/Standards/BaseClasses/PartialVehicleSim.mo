within BobLib.Experiments.Standards.BaseClasses;

partial model PartialVehicleSim

  import Modelica.Units.SI;
  import Modelica.Constants.pi;

  extends BobLib.Experiments.Standards.BaseClasses.PartialElectricVehicle(initialVel = 15,
    redeclare replaceable BobLib.DriverEnvironments.ExternalDriver driverEnvironment(
      includeManeuverMeasurements = true, steeringCenter = driver.steeringCenter, steeringRange = driver.steeringRange)
      constrainedby BobLib.DriverEnvironments.ExternalDriver);

  // Safety termination monitors
  parameter Boolean terminateOnTireLift = true "Terminate the maneuver if a tire reaches the lift threshold" annotation(choices(checkBox = true),
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Force tireLiftTerminateLoad = 75.0 "Tire normal load threshold used for hard lift termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter Boolean terminateOnSpinout = true "Terminate the maneuver if body sideslip indicates loss of directional control" annotation(choices(checkBox = true),
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Angle sideslipTerminate = 20*pi/180 "Absolute body sideslip threshold used for spinout termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));
  parameter SI.Time spinoutHoldDuration = 0.02 "Duration the sideslip threshold must remain true before spinout termination" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Safety"));

  // Open-loop lateral-linearity monitor
  parameter Boolean enableLinearityTermination = true "Terminate the open-loop ramp when steering response leaves the linear region" annotation(choices(checkBox = true),
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
  parameter SI.Time settleTimeout = 3.0 "Maximum wait after the ramp ends before reporting a settling timeout" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Settling"));

  // Closed-loop steady-state lateral-acceleration monitor
  parameter SI.Time steadyStateHoldDuration = 1.0 "Continuous duration required for closed-loop steady-state acceptance" annotation(
    Evaluate = false, Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));

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
  parameter SI.Time steadyStateSettleTimeout = 20.0 "Maximum wait after the closed-loop target reaches its final value" annotation(
    Evaluate = false,
    Dialog(tab = "Maneuver Monitors", group = "Steady-State Ay"));

  output Integer terminationStatus "Maneuver outcome: 0 running, 1 settled, 2 linearity limit, 3 settling timeout, 4 tire-load limit, 5 sideslip limit";

  // Maneuver diagnostics forwarded from StandardDriver
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

  replaceable BobLib.Controllers.VCU vcu(pVCU = pVehicle.controller) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Controller Models"),
    Placement(transformation(origin = {130, 50}, extent = {{-10, -10}, {10, 10}}))) constrainedby BobLib.Controllers.Interfaces.Base "Standard vehicle control unit";

  replaceable BobLib.Drivers.StandardDriver driver(
    targetVel = initialVel,
    driveTorqueCapacity = pVehicle.controller.tau_max*pVehicle.controller.finalDriveRatio,
    brakeTorqueCapacity = pVehicle.controller.mechanicalBrakeTorqueLimit)
    constrainedby BobLib.Drivers.StandardDriver annotation(choicesAllMatching = true,
      Placement(transformation(origin = {-40, 60}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnable(k = true)
    annotation(Placement(transformation(origin = {-140, 80}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));

  // Shared vehicle signal namespace

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

  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus
    "Typed driverBus signal contract" annotation(
      Placement(transformation(origin = {-180, 80}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(driverBus, controlBus.driverBus) annotation(
    Line(points = {{-180, 80}, {-200, 80}, {-200, 30}}, color = {255, 204, 51}, thickness = 0.5));
  assert(steadyStateHoldDuration > 0 and steadyStateSettleTimeout > steadyStateHoldDuration and
    tireLiftTerminateLoad >= 0 and sideslipTerminate > 0,
    "BOBLIB_INVALID_MONITOR_CONFIG: require positive hold and sideslip limit, timeout greater than hold, and nonnegative tire-load limit");

  // Report limit exits separately from accepted steady-state results. Limit
  // events end normally so callers can retain the last valid result sample.
  terminationStatus = if (driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnTireLift and time > driver.steerStart and minTireNormalLoad <= tireLiftTerminateLoad then 4
    elseif(driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnSpinout and t_spinout_hit > 0 and time > t_spinout_hit + spinoutHoldDuration then 5
    elseif(driver.useMode == driver.MODE_OPEN_LOOP_RAMP and rampEnding and time > t_ramp_end_hit + driver.handwheelRampStopDuration + settleTimeout) or
      (driver.useMode == driver.MODE_STEADY_STATE_AY and steadyStateTargetReached and time > driver.steerStart + steadyStateAyRampDuration + steadyStateSettleTimeout) then 3
        elseif driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and linearityGainLossFraction >= linearityNonlinearityFraction and t_linearity_limit_hit > 0 and time > t_linearity_limit_hit + linearityHoldDuration then 2
    elseif(driver.useMode == driver.MODE_OPEN_LOOP_RAMP and t_qss_hit > 0 and time > t_qss_hit + steadyHoldDuration) or
      (driver.useMode == driver.MODE_STEADY_STATE_AY and t_steady_state_hit > 0 and time > t_steady_state_hit + steadyStateHoldDuration) or
      (driver.useMode == driver.MODE_STEP_STEER and t_yawVel_hit > 0 and time > t_yawVel_hit + 0.1) then 1 else 0;
  frSteerCmd = driver.frSteerCmd;
  handwheelRampCmd = driver.handwheelRampCmd;
  handwheelRateCmd = driver.handwheelRateCmd;
  handwheelRampDirection = driver.handwheelRampDirection;
  minTireNormalLoad = driver.minTireNormalLoad;
  tireNormalLoadStopXi = driver.tireNormalLoadStopXi;
  tireNormalLoadRateXi = driver.tireNormalLoadRateXi;
  tireNormalLoadRateScale = driver.tireNormalLoadRateScale;
  rampEnding = driver.rampEnding;
  steerSine = driver.steerSine;
  steerStep = driver.steerStep;
  steadyStateAyRampDuration = driver.steadyStateAyRampDuration;
  steadyStateAyRampXi = driver.steadyStateAyRampXi;
  steadyStateAyCommand = driver.steadyStateAyCommand;
  steadyStateAyError = driver.steadyStateAyError;
  steadyStateAyIntegral = driver.steadyStateAyIntegral;
  steadyStateSteerCmd = driver.steadyStateSteerCmd;
  steadyStateTargetReached = driver.steadyStateTargetReached;
  linearityGainRatio = if linearityReferenceValid then noEvent(abs(linearityLocalLateralGain)/max(abs(linearityReferenceLateralGain), 1e-6))
    else 1;
  linearityGainLossFraction = if driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and noEvent(abs(accY) >= linearityReferenceAy + max(0, linearityEvaluationAyMargin)) then noEvent(max(0, 1 - linearityGainRatio))
    else 0;

  when sample(driver.steerStart, driver.linearitySlopeSamplePeriod) then
    if driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and time >= driver.steerStart then
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

    if driver.useMode == driver.MODE_STEADY_STATE_AY and time >= driver.steerStart then
      steadyStateSpeedError = driver.targetVel - chassis.vehicleSpeed;

      if pre(steadyStateSampleValid) then
        steadyStateAyRate = (accY - pre(steadyStateSampleAy))/max(driver.linearitySlopeSamplePeriod, 1e-6);
        steadyStateYawRateDerivative = (yawVel - pre(steadyStateSampleYawVel))/max(driver.linearitySlopeSamplePeriod, 1e-6);
        steadyStateSideslipRate = (sideslip - pre(steadyStateSampleSideslip))/max(driver.linearitySlopeSamplePeriod, 1e-6);
        steadyStateRollRate = (roll - pre(steadyStateSampleRoll))/max(driver.linearitySlopeSamplePeriod, 1e-6);
        steadyStateHandwheelRate = (handwheelAngle - pre(steadyStateSampleHandwheel))/max(driver.linearitySlopeSamplePeriod, 1e-6);
      else
        steadyStateAyRate = pre(steadyStateAyRate);
        steadyStateYawRateDerivative = pre(steadyStateYawRateDerivative);
        steadyStateSideslipRate = pre(steadyStateSideslipRate);
        steadyStateRollRate = pre(steadyStateRollRate);
        steadyStateHandwheelRate = pre(steadyStateHandwheelRate);
      end if;
      steadyStateConditionsMet = steadyStateTargetReached and abs(steadyStateAyCommand - accY) <= steadyStateAyTolerance and abs(driver.targetVel - chassis.vehicleSpeed) <= steadyStateSpeedTolerance and abs(steadyStateAyRate) <= steadyStateAyRateTolerance and abs(steadyStateYawRateDerivative) <= steadyStateYawRateDerivativeTolerance and abs(steadyStateSideslipRate) <= steadyStateSideslipRateTolerance and abs(steadyStateRollRate) <= steadyStateRollRateTolerance and abs(steadyStateHandwheelRate) <= handwheelRateTol;
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

  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and linearityGainLossFraction >= linearityNonlinearityFraction and pre(t_linearity_limit_hit) < 0 then
    t_linearity_limit_hit = time;
  elsewhen driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and linearityReferenceValid and linearityGainLossFraction < linearityNonlinearityFraction then
    t_linearity_limit_hit = -1;
  end when;

  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and enableLinearityTermination and t_linearity_limit_hit > 0 and time > t_linearity_limit_hit + linearityHoldDuration then
    terminate("Reached steering lateral-gain loss threshold");
  end when;

  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and rampEnding and not pre(rampEnding) then
    t_ramp_end_hit = time;
  end when;

  // Open-loop QSS detection starts after the normal-load floor ends the
  // handwheel ramp, then waits for yaw and steering rates to flatten.
  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and rampEnding and abs(der(yawVel)) < der_yawVelTol and abs(der(handwheelAngle)) < handwheelRateTol and pre(t_qss_hit) < 0 then
    t_qss_hit = time;
  elsewhen driver.useMode == driver.MODE_OPEN_LOOP_RAMP and rampEnding and (abs(der(yawVel)) >= der_yawVelTol or abs(der(handwheelAngle)) >= handwheelRateTol) then
    t_qss_hit = -1;
  end when;

  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and t_qss_hit > 0 and time > t_qss_hit + steadyHoldDuration then
    terminate("Reached open-loop ramp-steer QSS plateau");
  end when;

  when driver.useMode == driver.MODE_OPEN_LOOP_RAMP and rampEnding and time > t_ramp_end_hit + driver.handwheelRampStopDuration + settleTimeout then
    terminate(
      "BOBLIB_SETTLING_TIMEOUT: ramp-steer did not reach QSS at t=" + String(time) + " s; check controller response and steering/load calibration.");
  end when;

  when steadyStateConditionsMet and pre(t_steady_state_hit) < 0 then
    t_steady_state_hit = time;
  elsewhen driver.useMode == driver.MODE_STEADY_STATE_AY and not steadyStateConditionsMet then
    t_steady_state_hit = -1;
  end when;

  when driver.useMode == driver.MODE_STEADY_STATE_AY and t_steady_state_hit > 0 and time > t_steady_state_hit + steadyStateHoldDuration then
    terminate("Reached closed-loop steady-state lateral-acceleration target");
  end when;

  when driver.useMode == driver.MODE_STEADY_STATE_AY and steadyStateTargetReached and time > driver.steerStart + steadyStateAyRampDuration + steadyStateSettleTimeout then
    terminate("BOBLIB_SETTLING_TIMEOUT: t=" + String(time) +
      " s; target=" + String(steadyStateAyCommand) + " m/s2, actual=" + String(accY) +
      "; abs(error)/limit=" + String(abs(steadyStateAyError)) + "/" + String(steadyStateAyTolerance) +
      "; abs(ayRate)/limit=" + String(abs(steadyStateAyRate)) + "/" + String(steadyStateAyRateTolerance) +
      "; abs(yawRateDerivative)/limit=" + String(abs(steadyStateYawRateDerivative)) + "/" + String(steadyStateYawRateDerivativeTolerance) +
      "; abs(speedError)/limit=" + String(abs(steadyStateSpeedError)) + "/" + String(steadyStateSpeedTolerance) +
      "; required hold=" + String(steadyStateHoldDuration) + " s. Check sampled feedback, controller tuning, and numerical convergence.");
  end when;

  when (driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnTireLift and time > driver.steerStart and minTireNormalLoad <= tireLiftTerminateLoad then
    terminate("BOBLIB_TIRE_LOAD_LIMIT: t=" + String(time) + " s; minimum=" + String(minTireNormalLoad) +
      " N, limit=" + String(tireLiftTerminateLoad) + " N; FL=" + String(Fz_FL) + ", FR=" + String(Fz_FR) +
      ", RL=" + String(Fz_RL) + ", RR=" + String(Fz_RR) +
      " N. Maneuver exceeds this vehicle load margin; check roll stiffness distribution, CG, geometry, and demand.");
  end when;

  when (driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnSpinout and time > driver.steerStart and abs(sideslip) >= sideslipTerminate and pre(t_spinout_hit) < 0 then
    t_spinout_hit = time;
  elsewhen (driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnSpinout and abs(sideslip) < sideslipTerminate then
    t_spinout_hit = -1;
  end when;

  when (driver.useMode == driver.MODE_OPEN_LOOP_RAMP or driver.useMode == driver.MODE_STEADY_STATE_AY) and terminateOnSpinout and t_spinout_hit > 0 and time > t_spinout_hit + spinoutHoldDuration then
    terminate("BOBLIB_SIDESLIP_LIMIT: t=" + String(time) + " s; abs(sideslip)=" + String(abs(sideslip)) +
      " rad, limit=" + String(sideslipTerminate) + " rad. Check tire calibration, axle balance, and maneuver demand.");
  end when;

  when driver.useMode == driver.MODE_STEP_STEER and time > driver.steerStart and abs(der(yawVel)) < der_yawVelTol and pre(t_yawVel_hit) < 0 then
    t_yawVel_hit = time;
  elsewhen driver.useMode == driver.MODE_STEP_STEER and abs(der(yawVel)) >= der_yawVelTol then
    t_yawVel_hit = -1;
  end when;

  when driver.useMode == driver.MODE_STEP_STEER and t_yawVel_hit > 0 and time > t_yawVel_hit + 0.1 then
    terminate("Reached open-loop step-steer steady-state: der(yawVel) below tolerance (held 0.1s)");
  end when;
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{-200, 30}, {131, 30}, {131, 40}, {130, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(driver.driverInterface, driverEnvironment.driverInterface) annotation(
    Line(points = {{-20, 60}, {20, 60}}, color = {85, 85, 255}, thickness = 0.5));
  connect(inverterEnable.y, driverBus.inverterEnable) annotation(
    Line(points = {{-146.6, 80}, {-180, 80}}, color = {255, 0, 255}));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -180}, {180, 120}})),
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
Partial model <code>PartialVehicleSim</code> is the shared full-vehicle simulation template.
</p>
<p>
It exposes the complete replaceable vehicle stack: chassis, brakes, driveline,
transmission, electric drive, power electronics, energy storage, VCU, road,
atmosphere, aero, and world. The template owns plant bus wiring and maneuver
termination monitors. <code>Drivers.StandardDriver</code> owns the standard
autonomous maneuver commands, speed-control gains, target velocity, and pedal intent. The VCU owns electric-drive limits and brake blending; this experiment supplies EV ready-to-drive. Subsystem adapters publish the
measurements and commands they own on the shared VehicleInterfaces control bus,
and subscribers choose the fields they need.
</p>
<p>
The maneuver selector supports open-loop ramp steer, sine steer, step steer,
and closed-loop steady-state lateral-acceleration mode. In
<code>driver.useMode = 3</code>, <code>StandardDriver</code> ramps
<code>driver.targetAy</code>, uses a handwheel-angle PI controller to drive
measured <code>accY</code> to that target, and uses its longitudinal PI controller to request accelerator/brake intent. The template terminates only
after the lateral acceleration, speed, yaw rate, sideslip, roll, and
handwheel-rate settle criteria have held for
<code>steadyHoldDuration</code>.
</p>
</html>"));
end PartialVehicleSim;
