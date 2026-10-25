within BobLib.Experiments.Standards.BaseClasses;

partial model PartialVehicleFMI

  import Modelica.Units.SI;

  extends BobLib.Experiments.Standards.BaseClasses.PartialElectricVehicle(
    initialVel = 0,
    redeclare replaceable BobLib.DriverEnvironments.CommandedDriveByWire driverEnvironment(
      exactSteering = false, steeringFilterFrequency = steeringFilterFrequency)
      constrainedby BobLib.DriverEnvironments.Interfaces.Commanded);

  parameter SI.Frequency steeringFilterFrequency = 50
    "MSL position filter cutoff for sampled external steering commands" annotation(Dialog(group = "Operating settings"));

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad",
    start = 0) "Commanded steering-wheel angle" annotation(
      Placement(
        transformation(origin = {-200, 110}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput acceleratorPedalCommand(
    unit = "1",
    min = 0,
    max = 1,
    start = 0) "Commanded accelerator pedal position" annotation(
      Placement(
        transformation(origin = {-200, 85}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 45}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput brakePedalCommand(
    unit = "1",
    min = 0,
    max = 1,
    start = 0) "Commanded brake pedal position" annotation(
      Placement(
        transformation(origin = {-200, 60}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 20}, extent = {{-10, -10}, {10, 10}})));

  output SI.Velocity vehicleSpeed;

  replaceable BobLib.Controllers.VCU vcu(pVCU = pVehicle.controller) annotation(
    choicesAllMatching = true,
    Dialog(group = "Controller Models"),
    Placement(transformation(origin = {130, 50}, extent = {{-10, -10}, {10, 10}}))) constrainedby BobLib.Controllers.Interfaces.Base "Vehicle control unit";

protected

  Modelica.Blocks.Sources.BooleanConstant inverterEnableCommand(k = true)
    "Internal EV ready-to-drive / inverter-enable command" annotation(
    Placement(transformation(origin = {-110, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));

  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus
    "Typed driverBus signal contract" annotation(
      Placement(transformation(origin = {-150, 40}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(driverBus, controlBus.driverBus) annotation(
    Line(points = {{-150, 40}, {-170, 40}, {-170, 30}, {-200, 30}}, color = {255, 204, 51}, thickness = 0.5));
  vehicleSpeed = chassis.vehicleSpeed;
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{-200, 30}, {131, 30}, {131, 40}, {130, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(steeringAngleCommand, driverEnvironment.steeringAngleCommand) annotation(
    Line(points = {{-200, 110}, {-40, 110}, {-40, 76}, {18, 76}}, color = {0, 0, 127}));
  connect(acceleratorPedalCommand, driverEnvironment.acceleratorPedalCommand) annotation(
    Line(points = {{-200, 85}, {-30, 85}, {-30, 70}, {18, 70}}, color = {0, 0, 127}));
  connect(brakePedalCommand, driverEnvironment.brakePedalCommand) annotation(
    Line(points = {{-200, 60}, {18, 60}, {18, 64}}, color = {0, 0, 127}));
  connect(inverterEnableCommand.y, driverBus.inverterEnable) annotation(
    Line(points = {{-115.5, 40}, {-150, 40}}, color = {255, 0, 255}));
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
      variableFilter = "time|.*Command|vehicleSpeed|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|vcu\\..*|brakes\\..*"),
    Documentation(info = "<html>
<p>
Partial model <code>PartialVehicleFMI</code> is the shared full-vehicle assembly
for FMI export, driver-in-the-loop, and early software-in-the-loop usage.
</p>
<p>
The model exposes only steering wheel angle, accelerator pedal, and brake pedal
at its boundary. An internal signal holds the EV ready-to-drive state. Battery,
VCU, inverter, motor, driveline, brakes, chassis, aero, atmosphere, road, and
world remain preconfigured inside the assembly and communicate through the
internal VehicleInterfaces control bus.
</p>
<p>External steering commands pass through MSL's second-order position filter
(50 Hz by default). This supplies continuous steering velocity and acceleration
without requiring derivatives of host-supplied samples. Use the same cutoff in
native and FMI comparisons; the commanded and actual steering angles differ
during transients.</p>
<p>
By default the VCU speed-control paths are disabled so driver pedals produce
rear-axle drive torque and mechanical brake requests through the same bus
publish/subscribe path used by autonomous maneuver simulations. Direct driver
motor-torque and regenerative-torque pins are intentionally absent from this
base vehicle; those can be opened later by replacing the driver environment
with an explicit EV drive-by-wire adapter if a DIL rig needs them.
</p>
</html>"));
end PartialVehicleFMI;
