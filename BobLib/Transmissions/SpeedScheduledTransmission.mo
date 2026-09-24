within BobLib.Transmissions;
model SpeedScheduledTransmission

  "Multi-ratio transmission that selects a gear from vehicle speed"
  extends VehicleInterfaces.Icons.Transmission;
  extends VehicleInterfaces.Transmissions.Interfaces.Base;

  import SI = Modelica.Units.SI;

  parameter Integer nGears(min = 1) = 1
    "Number of forward gears";
  parameter Real gearRatios[nGears] = {3.31}
    "Gear ratios ordered from lowest gear (highest ratio) to highest gear";
  parameter SI.Velocity upshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear upshifts to the next";
  parameter SI.Velocity downshiftSpeeds[nGears - 1] = fill(0.0, nGears - 1)
    "Vehicle speed at which each gear downshifts to the previous";
  parameter Integer initialGear(min = 1, max = nGears) = 1
    "Gear engaged at initialization";
  parameter SI.Radius wheelRadius(min = 1e-6) = 0.2045
    "Loaded wheel radius used to estimate vehicle speed from output speed";

  BobLib.Transmissions.Internal.SpeedGearScheduler scheduler(
    nGears = nGears,
    gearRatios = gearRatios,
    upshiftSpeeds = upshiftSpeeds,
    downshiftSpeeds = downshiftSpeeds,
    initialGear = initialGear) annotation(
      Placement(transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Transmissions.Internal.VariableRatioGear gear annotation(
    Placement(transformation(extent = {{-24, -24}, {24, 24}})));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor outputSpeed annotation(
      Placement(transformation(origin = {58, 48}, extent = {{10, -10}, {-10, 10}})));

  output SI.AngularVelocity w_out "Transmission output speed";
  output SI.Velocity vehicleSpeedEstimate "Estimated vehicle speed";
  output Integer selectedGear "Engaged gear index";
  output Real selectedRatio "Engaged gear ratio";

protected
  VehicleInterfaces.Interfaces.TransmissionBus transmissionBus annotation(
    Placement(transformation(extent = {{-60, 50}, {-40, 70}})));

  Modelica.Blocks.Sources.RealExpression vehicleSpeedSignal(
    y = vehicleSpeedEstimate) annotation(
      Placement(transformation(origin = {-78, 40}, extent = {{-10, -6}, {10, 6}})));

  Modelica.Blocks.Sources.RealExpression gearRatioBusSignal(
    y = selectedRatio) "Engaged gear ratio published to the transmission bus" annotation(
      Placement(transformation(origin = {-8, 66}, extent = {{-8, -4}, {8, 4}})));

equation
  w_out = outputSpeed.w;
  // With Option A gearing the full reduction is folded into gearRatios, so the
  // transmission output shaft turns at wheel speed. Vehicle speed is therefore
  // just the wheel-speed times the loaded radius; no final drive is involved.
  vehicleSpeedEstimate = w_out*wheelRadius;
  selectedGear = scheduler.gear;
  selectedRatio = scheduler.ratio;

  connect(vehicleSpeedSignal.y, scheduler.vehicleSpeed) annotation(
    Line(points = {{-67, 40}, {-51, 40}}, color = {0, 0, 127}));
  connect(scheduler.ratio, gear.ratio) annotation(
    Line(points = {{-29, 40}, {0, 40}, {0, 24}}, color = {0, 0, 127}));

  connect(controlBus.transmissionBus, transmissionBus) annotation(
    Line(points = {{-100, 60}, {-50, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(engineFlange.flange, gear.inputFlange) annotation(
    Line(points = {{-100, 0}, {-24, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(gear.outputFlange, drivelineFlange.flange) annotation(
    Line(points = {{24, 0}, {100, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(outputSpeed.flange, gear.outputFlange) annotation(
    Line(points = {{68, 48}, {78, 48}, {78, 0}, {24, 0}}, color = {0, 0, 0}));
  connect(outputSpeed.w, transmissionBus.outputSpeed) annotation(
    Line(points = {{47, 48}, {30, 48}, {30, 60}, {-50, 60}}, color = {0, 0, 127}));
  connect(gearRatioBusSignal.y, transmissionBus.gearRatio) annotation(
    Line(points = {{0.8, 66}, {30, 66}, {30, 60}, {-50, 60}}, color = {0, 0, 127}));

  annotation(Documentation(info = "<html>
<p>
Model <code>SpeedScheduledTransmission</code> is a multi-ratio transmission
adapter for the neutral VehicleInterfaces transmission contract. It goes beyond
a single planetary/fixed reduction: given the current vehicle speed it selects
the appropriate discrete gear ratio and applies it through a variable-ratio
gear core.
</p>
<p>
Consistent with the package boundary rules, the public model owns the VI
engine/driveline flanges and control-bus publishing, while the reusable shift
logic (<code>Internal.SpeedGearScheduler</code>) and gear relation
(<code>Internal.VariableRatioGear</code>) live one level deeper. Set
<code>nGears = 1</code> to recover single-speed behaviour equivalent to
<code>FixedRatioTransmission</code>.
</p>
<p>
This adapter assumes Option A gearing: the complete reduction (primary, gear,
and sprocket/final drive) is folded into <code>gearRatios</code>, so the
transmission output shaft turns at wheel speed and the downstream driveline
final drive is unity. Vehicle speed used for scheduling is therefore just
<code>wheelRadius</code> times the output-shaft speed.
</p>
</html>"));
end SpeedScheduledTransmission;
