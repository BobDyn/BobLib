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
      Placement(transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

protected
  parameter SI.Density rho = ambientPressure/(R*T);

  Modelica.Blocks.Interfaces.RealOutput windVelocityWorldBusSignal[3](
    each quantity = "Velocity",
    each unit = "m/s") "Wind velocity published to atmosphereBus";
  Modelica.Blocks.Interfaces.RealOutput airDensityBusSignal(
    quantity = "Density",
    unit = "kg/m3") "Air density published to atmosphereBus";
  Modelica.Blocks.Interfaces.RealOutput airTemperatureBusSignal(
    quantity = "ThermodynamicTemperature",
    unit = "K") "Air temperature published to atmosphereBus";
  Modelica.Blocks.Interfaces.RealOutput relativeHumidityBusSignal(unit = "1")
    "Relative humidity published to atmosphereBus";
  Modelica.Blocks.Interfaces.RealOutput pressureBusSignal(
    quantity = "Pressure",
    unit = "Pa") "Ambient pressure published to atmosphereBus";

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
  windVelocityWorldBusSignal = v;
  airDensityBusSignal = rho;
  airTemperatureBusSignal = T;
  relativeHumidityBusSignal = h;
  pressureBusSignal = ambientPressure;

  connect(windVelocityWorldBusSignal, atmosphereBus.windVelocityWorld);
  connect(airDensityBusSignal, atmosphereBus.airDensity);
  connect(airTemperatureBusSignal, atmosphereBus.airTemperature);
  connect(relativeHumidityBusSignal, atmosphereBus.relativeHumidity);
  connect(pressureBusSignal, atmosphereBus.pressure);

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
