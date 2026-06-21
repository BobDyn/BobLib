within BobLib.Controllers;

model StandardVCU

  "Standard vehicle-simulation VCU with maneuver command generation"
  extends BobLib.Controllers.VCU;

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;

  // Maneuver mode selection
  constant Integer MODE_OPEN_LOOP_RAMP = 0;
  constant Integer MODE_OPEN_LOOP_SINE = 1;
  constant Integer MODE_STEP_STEER = 2;
  constant Integer MODE_STEADY_STATE_AY = 3;
  parameter Integer useMode = MODE_OPEN_LOOP_RAMP
    "0 - open-loop ramp steer; 1 - open-loop sinusoidal steer; 2 - step steer; 3 - closed-loop steady-state lateral acceleration"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Mode"));
  final parameter Boolean openLoopAy = useMode == MODE_OPEN_LOOP_RAMP;
  final parameter Boolean steadyStateAy = useMode == MODE_STEADY_STATE_AY;
  final parameter Boolean ayManeuver = openLoopAy or steadyStateAy;

  // Shared maneuver timing and target
  parameter SI.Time steerStart = 2.0
    "Start time" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Shared"));
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
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));
  parameter SI.Force tireNormalLoadMin = 200.0
    "Immediate tire normal-load floor where the handwheel ramp ends"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Open-Loop Ramp Steer", enable = openLoopAy));

  // Closed-loop steady-state lateral-acceleration steering
  parameter Real steadyStateAyRampRate(unit = "m/s3") = 2.0
    "Rate used to ramp the closed-loop lateral-acceleration target"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Angle steadyStateMaxHandwheel = 120*pi/180
    "Closed-loop steady-state steering command limit"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter Real steadyStateAyGain(unit = "rad.s2/m") = 0.050
    "Closed-loop steering PI gain from lateral-acceleration error to handwheel angle"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Time steadyStateAyTi = 1.0
    "Closed-loop steering PI integral time"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));
  parameter SI.Time steadyStateSteerTimeConstant = 0.15
    "First-order lag applied to the closed-loop steering command"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Steady-State Ay", enable = steadyStateAy));

  // Step steer command
  parameter SI.Angle frRampSteerHeight = 5*pi/180
    "Ramp steer target angle"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Step Steer"));
  parameter SI.Time frRampSteerDuration = 0.001
    "Ramp steer duration"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Step Steer"));
  parameter SI.Time stepDuration = frRampSteerDuration
    "Step steer duration"
    annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Step Steer"));

  // Sine steer command
  parameter SI.Angle steerAmp = 6*pi/180
    "Amplitude" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Sine Steer"));
  parameter SI.Frequency steerFreq = 1.0
    "Frequency (Hz)" annotation(Evaluate = false, Dialog(tab = "Maneuver", group = "Sine Steer"));

  // Driver-environment command outputs
  Modelica.Blocks.Interfaces.RealOutput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad") "Commanded steering-wheel angle" annotation(
      Placement(transformation(origin = {120, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealOutput acceleratorPedalCommand(unit = "1")
    "Commanded accelerator pedal position" annotation(
      Placement(transformation(origin = {120, 60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealOutput brakePedalCommand(unit = "1")
    "Commanded brake pedal position" annotation(
      Placement(transformation(origin = {120, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.BooleanOutput inverterEnableCommand
    "Internal EV ready-to-drive / inverter-enable command" annotation(
      Placement(transformation(origin = {120, 20}, extent = {{-10, -10}, {10, 10}})));

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
  output Real tireNormalLoadStopXi(
    start = 0,
    fixed = true,
    stateSelect = StateSelect.always)
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
  output SI.Velocity steadyStateAyIntegral(
    start = 0,
    fixed = true,
    stateSelect = StateSelect.always)
    "Sampled integral state for the steady-state lateral-acceleration PI";
  output SI.Angle steadyStateSteerCmd(
    start = 0,
    fixed = true,
    stateSelect = StateSelect.always)
    "Filtered steady-state handwheel command";
  output Boolean steadyStateTargetReached
    "True once the steady-state lateral-acceleration target ramp is complete";

protected
  discrete Boolean rampEndingState(start = false, fixed = true);
  SI.Acceleration measuredAy;

  Modelica.Blocks.Interfaces.RealInput normalLoad_1BusTap(
    quantity = "Force",
    unit = "N")
    "Front-left normal load from chassisBus" annotation(
      Placement(transformation(origin = {-120, 100}, extent = {{-6, -6}, {6, 6}})));

  Modelica.Blocks.Interfaces.RealInput normalLoad_2BusTap(
    quantity = "Force",
    unit = "N")
    "Front-right normal load from chassisBus" annotation(
      Placement(transformation(origin = {-120, 92}, extent = {{-6, -6}, {6, 6}})));

  Modelica.Blocks.Interfaces.RealInput normalLoad_3BusTap(
    quantity = "Force",
    unit = "N")
    "Rear-left normal load from chassisBus" annotation(
      Placement(transformation(origin = {-120, 84}, extent = {{-6, -6}, {6, 6}})));

  Modelica.Blocks.Interfaces.RealInput normalLoad_4BusTap(
    quantity = "Force",
    unit = "N")
    "Rear-right normal load from chassisBus" annotation(
      Placement(transformation(origin = {-120, 76}, extent = {{-6, -6}, {6, 6}})));

  Modelica.Blocks.Interfaces.RealInput lateralAccelerationBusTap(
    quantity = "Acceleration",
    unit = "m/s2")
    "Body lateral acceleration from chassisBus" annotation(
      Placement(transformation(origin = {-120, 68}, extent = {{-6, -6}, {6, 6}})));

equation
  assert(
    useMode == MODE_OPEN_LOOP_RAMP or useMode == MODE_OPEN_LOOP_SINE or useMode == MODE_STEP_STEER or useMode == MODE_STEADY_STATE_AY,
    "StandardVCU.useMode must be 0 (open-loop ramp), 1 (open-loop sine), 2 (step steer), or 3 (closed-loop steady-state lateral acceleration).");

  measuredAy = lateralAccelerationBusTap;
  minTireNormalLoad = noEvent(min(min(normalLoad_1BusTap, normalLoad_2BusTap), min(normalLoad_3BusTap, normalLoad_4BusTap)));
  rampEnding = rampEndingState;

  der(tireNormalLoadStopXi) =

    if useMode == MODE_OPEN_LOOP_RAMP and enableNormalLoadSteerLimiter and rampEndingState and noEvent(tireNormalLoadStopXi < 1) then
      1/max(handwheelRampStopDuration, 1e-6)
    else
      0;
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

  der(steadyStateAyIntegral) = 0;
  der(steadyStateSteerCmd) = 0;

  when sample(0, linearitySlopeSamplePeriod) and useMode == MODE_STEADY_STATE_AY and time >= steerStart then
    reinit(
      steadyStateAyIntegral,
      pre(steadyStateAyIntegral) + steadyStateAyError*max(linearitySlopeSamplePeriod, 1e-6));
    reinit(
      steadyStateSteerCmd,
      pre(steadyStateSteerCmd) + min(1, max(linearitySlopeSamplePeriod, 1e-6)/max(steadyStateSteerTimeConstant, 1e-6))*(max(-steadyStateMaxHandwheel, min(steadyStateMaxHandwheel, steadyStateAyGain*(steadyStateAyError + (pre(steadyStateAyIntegral) + steadyStateAyError*max(linearitySlopeSamplePeriod, 1e-6))/max(steadyStateAyTi, 1e-6)))) - pre(steadyStateSteerCmd)));
  end when;

  when useMode == MODE_OPEN_LOOP_RAMP and time > steerStart and enableNormalLoadSteerLimiter and minTireNormalLoad <= tireNormalLoadMin and not pre(rampEndingState) then
    rampEndingState = true;
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
  acceleratorPedalCommand = 0;
  brakePedalCommand = 0;
  inverterEnableCommand = true;

  connect(controlBus.chassisBus.Fz_1, normalLoad_1BusTap) annotation(
    Line(points = {{0, -100}, {-120, -100}, {-120, 100}}, color = {0, 0, 127}));
  connect(controlBus.chassisBus.Fz_2, normalLoad_2BusTap) annotation(
    Line(points = {{0, -100}, {-120, -100}, {-120, 92}}, color = {0, 0, 127}));
  connect(controlBus.chassisBus.Fz_3, normalLoad_3BusTap) annotation(
    Line(points = {{0, -100}, {-120, -100}, {-120, 84}}, color = {0, 0, 127}));
  connect(controlBus.chassisBus.Fz_4, normalLoad_4BusTap) annotation(
    Line(points = {{0, -100}, {-120, -100}, {-120, 76}}, color = {0, 0, 127}));
  connect(controlBus.chassisBus.bodyAcceleration_2, lateralAccelerationBusTap) annotation(
    Line(points = {{0, -100}, {-120, -100}, {-120, 68}}, color = {0, 0, 127}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>StandardVCU</code> extends the public <code>VCU</code> adapter with
the standard vehicle-simulation maneuver policy. It owns the open-loop steering
waveforms, normal-load-limited ramp steer, closed-loop steady-state
lateral-acceleration steering PI, inactive driver pedal commands, and
ready-to-drive command used by <code>Experiments.Standards.VehicleSim</code>.
</p>
<p>
The model subscribes to chassis measurements through the expandable
<code>controlBus.chassisBus</code> and leaves the base VCU responsible for
publishing electric-drive, driveline, and mechanical-brake actuator requests.
</p>
</html>"));
end StandardVCU;
