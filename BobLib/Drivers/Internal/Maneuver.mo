within BobLib.Drivers.Internal;

model Maneuver

  "Standard driver policy and normalized VI commands"

  import Modelica.Units.SI;
  import Modelica.Constants.pi;

  extends VehicleInterfaces.Drivers.Interfaces.Base;
  extends VehicleInterfaces.Icons.Driver;

  // Maneuver mode selection
  constant Integer MODE_OPEN_LOOP_RAMP = 0;
  constant Integer MODE_OPEN_LOOP_SINE = 1;
  constant Integer MODE_STEP_STEER = 2;
  constant Integer MODE_STEADY_STATE_AY = 3;
  parameter Integer useMode = MODE_OPEN_LOOP_RAMP
    "0 - open-loop ramp steer; 1 - open-loop sinusoidal steer; 2 - step steer; 3 - closed-loop steady-state lateral acceleration"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Mode"));
  final parameter Boolean openLoopAy = useMode == MODE_OPEN_LOOP_RAMP "Select the open-loop lateral-acceleration ramp maneuver";
  final parameter Boolean steadyStateAy = useMode == MODE_STEADY_STATE_AY "Select closed-loop tracking of steady-state lateral acceleration";
  final parameter Boolean ayManeuver = openLoopAy or steadyStateAy "Enable lateral-acceleration maneuver logic for either ramp or steady-state mode";

  // Shared maneuver timing and target
  parameter SI.Time steerStart = 2.0
    "Time at which the selected steering maneuver begins" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Shared"));
  parameter SI.Acceleration targetAy = 18
    "Target lateral acceleration; open-loop ramp mode uses only its sign"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Shared", enable = ayManeuver));
  parameter SI.Time linearitySlopeSamplePeriod = 0.10
    "Sample period for finite-difference controller updates"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Shared", enable = ayManeuver));

  // Open-loop ramp steer command
  parameter SI.AngularVelocity handwheelRampRate = 0.14
    "Open-loop handwheel ramp rate"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));
  parameter SI.Time handwheelRampStopDuration = 0.18
    "Duration used to smoothly roll handwheel rate to zero after the load limit"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));
  parameter Boolean enableNormalLoadSteerLimiter = true
    "End the open-loop handwheel ramp when any tire reaches the load floor"
    annotation(choices(checkBox = true), Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));
  parameter SI.Force tireNormalLoadMin = 200.0
    "Immediate tire normal-load floor where the handwheel ramp ends"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));

  // Closed-loop steady-state lateral-acceleration steering
  parameter Real steadyStateAyRampRate(unit = "m/s3") = 2.0
    "Average ramp rate; the cubic target ramp has a peak rate 1.5 times this value"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Angle steadyStateMaxHandwheel = 120*pi/180
    "Closed-loop steady-state steering command limit"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter Real steadyStateAyGain(unit = "rad.s2/m") = 0.04
    "Closed-loop steering PI gain from lateral-acceleration error to handwheel angle"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Time steadyStateAyTi = 1.6
    "Closed-loop steering PI integral time"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Time steadyStateSteerTimeConstant = 0.15
    "First-order lag applied to the closed-loop steering command"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));

  // Step steer command
  parameter SI.Angle frRampSteerHeight = 5*pi/180
    "Handwheel angle reached by the step-steer maneuver"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Step Steer"));
  parameter SI.Time stepDuration = 0.001
    "Time taken to transition from zero to the step-steer handwheel target"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Step Steer"));

  // Sine steer command
  parameter SI.Angle steerAmp = 6*pi/180
    "Handwheel angle amplitude of the sinusoidal steering maneuver, in radians" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Sine Steer"));
  parameter SI.Frequency steerFreq = 1.0
    "Frequency of the sinusoidal steering maneuver, in hertz" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Sine Steer"));

  parameter SI.Velocity targetVel = 15 "Vehicle speed setpoint for the maneuver speed controller" annotation(Dialog(group = "Operating settings"));
  parameter Real velGain = 200 "Proportional gain converting speed error to requested wheel torque" annotation(Dialog(group = "Operating settings"));
  parameter SI.Time velTi = 1 "Integral time of the maneuver speed controller" annotation(Dialog(group = "Operating settings"));
  parameter SI.Torque driveTorqueCapacity = 240 "Available driven-wheel torque used to normalize accelerator demand" annotation(Dialog(group = "Operating settings"));
  parameter SI.Torque brakeTorqueCapacity = 1500 "Available mechanical braking torque used to normalize brake demand" annotation(Dialog(group = "Operating settings"));
  parameter Boolean enableDriveSpeedControl = true "Allow the speed controller to request positive drive torque" annotation(Dialog(group = "Operating settings"), choices(checkBox = true), Evaluate = false);
  parameter Boolean enableBrakeSpeedControl = true "Allow the speed controller to request braking torque" annotation(Dialog(group = "Operating settings"), choices(checkBox = true), Evaluate = false);
  parameter Real steeringCenter(unit = "1") = 0.5 "Normalized steering command corresponding to zero handwheel angle" annotation(Dialog(group = "Operating settings"));
  parameter SI.Angle steeringRange = 4*pi "Handwheel angle span corresponding to a unit change in normalized steering command" annotation(Dialog(group = "Operating settings"));
  output SI.Angle steeringAngleCommand;
  output Real acceleratorPedalCommand(unit = "1");
  output Real brakePedalCommand(unit = "1");

  // Maneuver diagnostics forwarded by vehicle templates
  output SI.Angle frSteerCmd
    "Final steering command issued by the standard maneuver controller";
  output SI.Angle handwheelRampCmd(
    start = 0,
    fixed = true,
    stateSelect = StateSelect.always)
    "Integrated open-loop handwheel ramp command";
  output SI.AngularVelocity handwheelRateCmd
    "Open-loop handwheel ramp rate command";
  output Real handwheelRampDirection
    "Signed open-loop handwheel direction";
  output SI.Force minTireNormalLoad
    "Minimum corner normal load subscribed from chassisBus";
  output Real tireNormalLoadStopXi
    "Smooth ramp-stop progress after the tire load floor is reached";
  output Real tireNormalLoadRateXi
    "Clamped smoothstep input for the normal-load ramp stop";
  output Real tireNormalLoadRateScale
    "Open-loop ramp-rate scale after the normal-load floor is reached";
  output Boolean rampEnding
    "True once the open-loop ramp has reached the tire load floor";
  output SI.Angle steerSine
    "Open-loop sine steering command";
  output SI.Angle steerStep
    "Open-loop step steering command";
  output SI.Time steadyStateAyRampDuration
    "Duration of the steady-state lateral-acceleration target ramp";
  output Real steadyStateAyRampXi
    "Normalized steady-state lateral-acceleration target ramp progress";
  output SI.Acceleration steadyStateAyCommand(start = 0)
    "Commanded lateral acceleration for steady-state mode";
  output Real steadyStateAyError
    "Measured lateral-acceleration error for steady-state steering control";
  discrete output SI.Velocity steadyStateAyIntegral(
    start = 0,
    fixed = true)
    "Sampled integral state for the steady-state lateral-acceleration PI";
  output SI.Angle steadyStateSteerCmd(
    start = 0,
    fixed = true,
    stateSelect = StateSelect.always)
    "Sampled first-order steady-state handwheel command";
  output Boolean steadyStateTargetReached
    "True once the steady-state lateral-acceleration target ramp is complete";

protected
  discrete SI.Velocity steadyStateIntegralCandidate(start = 0, fixed = true);
  discrete Boolean rampEndingState(start = false, fixed = true);
  discrete SI.Time rampEndTime(start = 0, fixed = true);
  SI.Acceleration measuredAy;

  Modelica.Blocks.Interfaces.RealInput normalLoad_1BusTap annotation(Placement(transformation(origin = {10, -20}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput normalLoad_2BusTap annotation(Placement(transformation(origin = {10, -40}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput normalLoad_3BusTap annotation(Placement(transformation(origin = {10, -60}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput normalLoad_4BusTap annotation(Placement(transformation(origin = {10, -80}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput lateralAccelerationBusTap annotation(Placement(transformation(origin = {10, -100}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput vehicleSpeedBusTap annotation(Placement(transformation(origin = {10, -120}, extent = {{-4, -4}, {4, 4}}, rotation = 180)));

  Modelica.Blocks.Continuous.LimPID speedPI(
    controllerType = Modelica.Blocks.Types.SimpleController.PI, k = velGain, Ti = velTi,
    yMax = driveTorqueCapacity, yMin = -brakeTorqueCapacity, Ni = 0.9,
    initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 0)
    annotation(Placement(transformation(origin = {-200, -80}, extent = {{-15, -15}, {15, 15}})));
  VehicleInterfaces.Mechanics.NormalisedRotational.Position steeringPosition annotation(Placement(transformation(origin = {-110, 100}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Mechanics.NormalisedTranslational.Position acceleratorPosition annotation(Placement(transformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}})));
  VehicleInterfaces.Mechanics.NormalisedTranslational.Position brakePosition annotation(Placement(transformation(origin = {-110, 20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.IntegerConstant gearboxMode(k = VehicleInterfaces.Types.GearMode.Drive) annotation(Placement(transformation(origin = {84, 120}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.IntegerConstant requestedGear(k = 0) annotation(Placement(transformation(origin = {84, 80}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.IntegerConstant ignition(k = VehicleInterfaces.Types.IgnitionSetting.On) annotation(Placement(transformation(origin = {84, 40}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.Constant speedTarget(k = targetVel) annotation(Placement(transformation(origin = {-285.6, -80}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.RealExpression steeringCommand(y = steeringCenter+steeringAngleCommand/steeringRange) annotation(Placement(transformation(origin = {-212.55, 100}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.RealExpression acceleratorCommand(y = acceleratorPedalCommand) annotation(Placement(transformation(origin = {-212.55, 60}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Sources.RealExpression brakeCommand(y = brakePedalCommand) annotation(Placement(transformation(origin = {-212.55, 20}, extent = {{-8, -8}, {8, 8}})));

equation
  assert(
    linearitySlopeSamplePeriod > 0 and stepDuration > 0,
    "Maneuver: sample period and step duration must be positive");
  assert(
    useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_OPEN_LOOP_SINE or useMode == MODE_STEP_STEER or useMode == MODE_STEADY_STATE_AY,
    "Maneuver.useMode must be 0 (open-loop ramp), 1 (open-loop sine), 2 (step steer), or 3 (closed-loop steady-state lateral acceleration).");

  assert(steadyStateAyGain > 0 and steadyStateAyGain < Modelica.Constants.inf and
    steadyStateAyTi > 0 and steadyStateAyTi < Modelica.Constants.inf and
    steadyStateSteerTimeConstant > 0 and steadyStateSteerTimeConstant < Modelica.Constants.inf and
    steadyStateMaxHandwheel > 0 and steadyStateMaxHandwheel < Modelica.Constants.inf and
    steadyStateAyRampRate > 0 and steadyStateAyRampRate < Modelica.Constants.inf,
    "BOBLIB_INVALID_DRIVER_CONFIG: steady-state PI gain, integral time, steering time constant, steering limit, and target ramp rate must be finite and positive");

  measuredAy = lateralAccelerationBusTap;
  minTireNormalLoad = noEvent(min(min(normalLoad_1BusTap, normalLoad_2BusTap), min(normalLoad_3BusTap, normalLoad_4BusTap)));
  rampEnding = rampEndingState;

  tireNormalLoadStopXi = if rampEndingState then
    noEvent(min(1, max(0, (time - rampEndTime)/max(handwheelRampStopDuration, 1e-6))))
      else 0;
  tireNormalLoadRateXi =
    noEvent(min(1, max(0, tireNormalLoadStopXi)));
  tireNormalLoadRateScale =
    noEvent(1 - (3*tireNormalLoadRateXi^2 - 2*tireNormalLoadRateXi^3));

  handwheelRampDirection =

    if noEvent(targetAy >= 0) then
      1
    else
      -1;
  handwheelRateCmd =

    if useMode == MODE_OPEN_LOOP_RAMP and noEvent(time >= steerStart) then
      handwheelRampDirection*handwheelRampRate*tireNormalLoadRateScale
    else
      0;
  der(handwheelRampCmd) = handwheelRateCmd;

  steadyStateAyRampDuration =
    noEvent(abs(targetAy)/max(steadyStateAyRampRate, 1e-6));
  steadyStateAyRampXi =

    if steadyStateAy and noEvent(time >= steerStart) then
      noEvent(min(1, max(0, (time - steerStart)/max(steadyStateAyRampDuration, 1e-6))))
    else
      0;
  steadyStateAyCommand =
    targetAy*noEvent(3*steadyStateAyRampXi^2 - 2*steadyStateAyRampXi^3);
  steadyStateAyError = steadyStateAyCommand - measuredAy;
  steadyStateTargetReached =
    steadyStateAy and noEvent(steadyStateAyRampXi >= 1);

  // Retain this angle state: removing it broke full-vehicle initialization in OMC 1.27.1.
  der(steadyStateSteerCmd) = 0;

  when sample(0, linearitySlopeSamplePeriod) and useMode == MODE_STEADY_STATE_AY and time >= steerStart then
    steadyStateIntegralCandidate = pre(steadyStateAyIntegral) + steadyStateAyError*linearitySlopeSamplePeriod;
    steadyStateAyIntegral = if
      (steadyStateAyError > 0 and steadyStateAyGain*(steadyStateAyError + steadyStateIntegralCandidate/steadyStateAyTi) > steadyStateMaxHandwheel) or
      (steadyStateAyError < 0 and steadyStateAyGain*(steadyStateAyError + steadyStateIntegralCandidate/steadyStateAyTi) < -steadyStateMaxHandwheel)
      then pre(steadyStateAyIntegral) else steadyStateIntegralCandidate;
    reinit(steadyStateSteerCmd, pre(steadyStateSteerCmd) +
      min(1, linearitySlopeSamplePeriod/steadyStateSteerTimeConstant)*
      (max(-steadyStateMaxHandwheel, min(steadyStateMaxHandwheel,
        steadyStateAyGain*(steadyStateAyError + steadyStateAyIntegral/steadyStateAyTi))) - pre(steadyStateSteerCmd)));
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP and time > steerStart and enableNormalLoadSteerLimiter and minTireNormalLoad <= tireNormalLoadMin and not pre(rampEndingState) then
    rampEndingState = true;
    rampEndTime = time;
  end when;

  steerSine =

    if noEvent(useMode == MODE_OPEN_LOOP_SINE and time > steerStart) then
      steerAmp*sin(2*pi*steerFreq*(time - steerStart))
    else
      0;
  steerStep =

    if noEvent(time > steerStart) then
      frRampSteerHeight*noEvent(min(1, max(0, (time - steerStart)/stepDuration)))
    else
      0;

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

  steeringAngleCommand = frSteerCmd;
  acceleratorPedalCommand = if enableDriveSpeedControl then noEvent(min(1, max(0, speedPI.y/driveTorqueCapacity)))
    else 0;
  brakePedalCommand = if enableBrakeSpeedControl then noEvent(min(1, max(0, -speedPI.y/brakeTorqueCapacity))) else 0;
  assert(
    driveTorqueCapacity>0 and brakeTorqueCapacity>0 and steeringRange>0,
    "Driver capacities and steering range must be positive");

  connect(driverInterface.Fz_1, normalLoad_1BusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -20}, {10, -20}}, color = {0, 0, 127}));
  connect(driverInterface.Fz_2, normalLoad_2BusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -40}, {10, -40}}, color = {0, 0, 127}));
  connect(driverInterface.Fz_3, normalLoad_3BusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -60}, {10, -60}}, color = {0, 0, 127}));
  connect(driverInterface.Fz_4, normalLoad_4BusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -80}, {10, -80}}, color = {0, 0, 127}));
  connect(driverInterface.bodyAcceleration_2, lateralAccelerationBusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -100}, {10, -100}}, color = {0, 0, 127}));
  connect(driverInterface.vehicleSpeed, vehicleSpeedBusTap) annotation(Line(points = {{100, 0}, {50, 0}, {50, -120}, {10, -120}}, color = {0, 0, 127}));
  connect(steeringPosition.flange_b, driverInterface.steeringWheel) annotation(Line(points = {{-100, 100}, {50, 100}, {50, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(acceleratorPosition.flange_b, driverInterface.acceleratorPedal) annotation(Line(points = {{-100, 60}, {50, 60}, {50, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(brakePosition.flange_b, driverInterface.brakePedal) annotation(Line(points = {{-100, 20}, {50, 20}, {50, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(gearboxMode.y, driverInterface.gearboxMode) annotation(Line(points = {{92.8, 120}, {96, 120}, {96, 0}, {100, 0}}, color = {255, 127, 0}));
  connect(requestedGear.y, driverInterface.requestedGear) annotation(Line(points = {{92.8, 80}, {96, 80}, {96, 0}, {100, 0}}, color = {255, 127, 0}));
  connect(ignition.y, driverInterface.ignition) annotation(Line(points = {{92.8, 40}, {96, 40}, {96, 0}, {100, 0}}, color = {255, 127, 0}));
  connect(speedTarget.y, speedPI.u_s) annotation(Line(points = {{-276.8, -80}, {-218, -80}}, color = {0, 0, 127}));
  connect(vehicleSpeedBusTap, speedPI.u_m) annotation(Line(points = {{10, -120}, {-200, -120}, {-200, -98}}, color = {0, 0, 127}));
  connect(steeringCommand.y, steeringPosition.phi_ref) annotation(Line(points = {{-203.75, 100}, {-122, 100}}, color = {0, 0, 127}));
  connect(acceleratorCommand.y, acceleratorPosition.position) annotation(Line(points = {{-203.75, 60}, {-122, 60}}, color = {0, 0, 127}));
  connect(brakeCommand.y, brakePosition.position) annotation(Line(points = {{-203.75, 20}, {-122, 20}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-320, -140}, {100, 140}}, preserveAspectRatio = true)),
    Documentation(info = "<html><p>Owns steering maneuvers and longitudinal speed control. Publishes normalized driver intent through VI DriverInterface; the environment maps that intent to actuator commands and the VCU owns torque limits and brake blending. steeringCenter and steeringRange must match the environment.</p><h4>Parameter setup</h4><p>Select <code>useMode</code> first: 0 ramp steer, 1 sinusoidal steer, 2 step steer, 3 closed-loop lateral acceleration. <code>steerStart</code> applies to all modes. For step steer, <code>frRampSteerHeight</code> is the handwheel target and <code>stepDuration</code> is the transition time. Steering angles use radians. The speed controller uses <code>targetVel</code>, <code>velGain</code> and <code>velTi</code>; drive and brake torque capacities normalize its requests into pedal commands. Maneuver-specific enable conditions in the dialog identify the applicable settings.</p>
<p>Closed-loop lateral acceleration uses a sampled PI controller with conditional integration at the steering limit. Its steering command follows a sampled first-order recurrence, not a continuous actuator lag. Retain the prescribed steering-angle state for full-vehicle initialization in OpenModelica. The target uses a cubic ramp of duration <code>abs(targetAy)/steadyStateAyRampRate</code>, so its peak ramp rate is 1.5 times the configured average. A target outside the vehicle's feasible envelope must trigger the experiment's safety or settling diagnostic, not be treated as converged.</p>
</html>"));
end Maneuver;
