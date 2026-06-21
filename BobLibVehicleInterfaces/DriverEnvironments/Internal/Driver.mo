within BobLibVehicleInterfaces.DriverEnvironments.Internal;

model Driver

  "Minimal driver-intent publisher for VehicleInterfaces driverBus signals"

  extends VehicleInterfaces.Icons.DriverEnvironment;

  extends VehicleInterfaces.DriverEnvironments.Interfaces.Base(
    final includeDriverSeat = false,
    final includeSteeringWheel = true,
    final includeAcceleratorPedal = false,
    final includeBrakePedal = false);

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad") "Commanded steering wheel angle" annotation(
    Placement(
      transformation(origin = {-120, 120}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput acceleratorPedalCommand(unit = "1")
    "Commanded accelerator pedal position" annotation(
    Placement(
      transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput brakePedalCommand(unit = "1")
    "Commanded brake pedal position" annotation(
    Placement(
      transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}),
      iconTransformation(origin = {-110, 20}, extent = {{-10, -10}, {10, 10}})));

protected
  Modelica.Mechanics.Rotational.Sources.Position steeringPosition(
    exact = true,
    w(start = 0)) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(steeringPosition.flange, steeringWheel) annotation(
    Line(points = {{80, 0}, {100, 0}}));
  connect(steeringAngleCommand, controlBus.driverBus.steeringWheelAngle) annotation(
    Line(points = {{-120, 120}, {100, 120}, {100, 60}}, color = {0, 0, 127}));
  connect(acceleratorPedalCommand, controlBus.driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-120, 80}, {0, 80}, {0, 120}, {100, 120}, {100, 60}}, color = {0, 0, 127}));
  connect(brakePedalCommand, controlBus.driverBus.brakePedalPosition) annotation(
    Line(points = {{-120, 40}, {0, 40}, {0, 120}, {100, 120}, {100, 60}}, color = {0, 0, 127}));
  connect(steeringAngleCommand, steeringPosition.phi_ref) annotation(
    Line(points = {{-120, 120}, {40, 120}, {40, 0}, {58, 0}}, color = {0, 0, 127}));

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
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 140}})));
end Driver;
