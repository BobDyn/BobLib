within BobLibVehicleInterfaces.Atmospheres;
model ConstantAtmosphere

  "Constant atmosphere that publishes measurements on an atmosphere bus"
  extends VehicleInterfaces.Icons.Atmosphere;
  extends VehicleInterfaces.Atmospheres.Interfaces.Base(
    redeclare function windVelocity = constantWindVelocity(windVelocity = v),
    redeclare function density = constantDensity(density = rho),
    redeclare function temperature = constantTemperature(T0 = T),
    redeclare function humidity = constantHumidity(h0 = h));

  import SI = Modelica.Units.SI;

  parameter SI.Velocity v[3] = zeros(3)
    "Wind velocity resolved in world frame";
  parameter SI.AbsolutePressure ambientPressure = 101300
    "Air pressure";
  parameter SI.Temperature T = 293.15
    "Air temperature";
  parameter Real h(unit = "1", min = 0, max = 1) = 0.5
    "Air humidity";
  constant Real R = 287.0512249529787
    "Gas constant for air";

  BobLibVehicleInterfaces.Atmospheres.Interfaces.AtmosphereBus atmosphereBus
    "Atmosphere signal bus" annotation(
      Placement(transformation(origin = {-100, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-149.5, 100.5}, extent = {{-39.5, -39.5}, {39.5, 39.5}})));

protected
  parameter SI.Density rho = ambientPressure/(R*T);

  Modelica.Blocks.Sources.RealExpression windVelocityWorldBusSignal[3](
    y = v) "Wind velocity published to atmosphereBus" annotation(
      Placement(transformation(origin = {30, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression airDensityBusSignal(
    y = rho) "Air density published to atmosphereBus" annotation(
      Placement(transformation(origin = {30, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression airTemperatureBusSignal(
    y = T) "Air temperature published to atmosphereBus" annotation(
      Placement(transformation(origin = {30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression relativeHumidityBusSignal(
    y = h) "Relative humidity published to atmosphereBus" annotation(
      Placement(transformation(origin = {30, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression pressureBusSignal(
    y = ambientPressure) "Ambient pressure published to atmosphereBus" annotation(
      Placement(transformation(origin = {30, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  function constantWindVelocity

    extends VehicleInterfaces.Atmospheres.Interfaces.windVelocityBase;
    input SI.Velocity windVelocity[3] = {0, 0, 0};

  algorithm
    v := windVelocity;
  end constantWindVelocity;

  function constantDensity

    extends VehicleInterfaces.Atmospheres.Interfaces.densityBase;
    input SI.Density density = 1.18;

  algorithm
    rho := density;
  end constantDensity;

  function constantTemperature

    extends VehicleInterfaces.Atmospheres.Interfaces.temperatureBase;
    input SI.Temperature T0 = 293;

  algorithm
    T := T0;
  end constantTemperature;

  function constantHumidity

    extends VehicleInterfaces.Atmospheres.Interfaces.humidityBase;
    input Real h0 = 0.5;

  algorithm
    h := h0;
  end constantHumidity;

equation

  connect(windVelocityWorldBusSignal.y, atmosphereBus.windVelocityWorld) annotation(
    Line(points = {{19, 40}, {-100, 40}}, color = {0, 0, 127}));
  connect(airDensityBusSignal.y, atmosphereBus.airDensity) annotation(
    Line(points = {{19, 20}, {-86, 20}, {-86, 40}, {-100, 40}}, color = {0, 0, 127}));
  connect(airTemperatureBusSignal.y, atmosphereBus.airTemperature) annotation(
    Line(points = {{19, 0}, {-86, 0}, {-86, 40}, {-100, 40}}, color = {0, 0, 127}));
  connect(relativeHumidityBusSignal.y, atmosphereBus.relativeHumidity) annotation(
    Line(points = {{19, -20}, {-86, -20}, {-86, 40}, {-100, 40}}, color = {0, 0, 127}));
  connect(pressureBusSignal.y, atmosphereBus.pressure) annotation(
    Line(points = {{19, -40}, {-86, -40}, {-86, 40}, {-100, 40}}, color = {0, 0, 127}));
  annotation(
    defaultComponentName = "atmosphere",
    defaultComponentPrefixes = "inner",
    Documentation(info = "<html>
<p>
Model <code>ConstantAtmosphere</code> is a BobLib constant-atmosphere adapter.
It preserves the VehicleInterfaces atmosphere function contract and publishes
wind velocity, density, temperature, humidity, and pressure on its
<code>atmosphereBus</code> connector.
</p>
<p>
The bus is the public signal surface for atmosphere-owned measurements. Aero
models subscribe to it and combine atmosphere-owned wind with their chassis
frame velocity to calculate relative airspeed locally. Although this
implementation derives its values from parameters, the bus carries signals so
future atmosphere models can vary pressure, temperature, density, humidity, or
wind without changing aero wiring.
</p>
</html>"));
end ConstantAtmosphere;