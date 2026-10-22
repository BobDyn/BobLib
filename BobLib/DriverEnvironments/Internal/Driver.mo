within BobLib.DriverEnvironments.Internal;

model Driver

  "Minimal driver-intent publisher for VehicleInterfaces driverBus signals"

  extends VehicleInterfaces.Icons.DriverEnvironment;

  extends BobLib.DriverEnvironments.Interfaces.Commanded;

protected
  Modelica.Mechanics.Rotational.Sources.Position steeringPosition(
    exact = exactSteering,
    f_crit = steeringFilterFrequency,
    phi(stateSelect = if exactSteering then StateSelect.default else StateSelect.always),
    w(start = 0, stateSelect = if exactSteering then StateSelect.default else StateSelect.always)) annotation(
    Placement(transformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus
    "Typed driverBus signal contract" annotation(
      Placement(transformation(origin = {60, 60}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(driverBus, controlBus.driverBus) annotation(
    Line(points = {{60, 60}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(steeringPosition.flange, steeringWheel) annotation(
    Line(points = {{-90, -30}, {100, -30}, {100, 0}}));
  connect(steeringAngleCommand, driverBus.steeringWheelAngle) annotation(
    Line(points = {{-200, -30}, {-150, -30}, {-150, -10}, {90, -10}, {90, 60}, {60, 60}}, color = {0, 0, 127}));
  connect(acceleratorPedalCommand, driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-200, 60}, {60, 60}}, color = {0, 0, 127}));
  connect(brakePedalCommand, driverBus.brakePedalPosition) annotation(
    Line(points = {{-200, 30}, {80, 30}, {80, 60}, {60, 60}}, color = {0, 0, 127}));
  connect(steeringAngleCommand, steeringPosition.phi_ref) annotation(
    Line(points = {{-200, -30}, {-112, -30}}, color = {0, 0, 127}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>Driver</code> is the minimal BobLib driver-intent publisher. It
receives only steering, accelerator, and brake commands, publishes those
standard VehicleInterfaces <code>driverBus</code> fields, and drives the
standard steering-wheel flange.
</p>
<p>
Transmission, ignition, EV, and IC command publishers should be composed
alongside this model rather than added here, so basic driver intent stays
independent of powertrain architecture.
</p>
</html>"),
    Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true)));
end Driver;
