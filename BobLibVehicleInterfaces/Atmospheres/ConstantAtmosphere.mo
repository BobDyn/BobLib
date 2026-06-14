within BobLibVehicleInterfaces.Atmospheres;
model ConstantAtmosphere
  "Constant atmosphere with explicit signal outputs for aero wiring"
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

  Modelica.Blocks.Interfaces.RealOutput windVelocityWorld[3](
    each quantity = "Velocity",
    each unit = "m/s") "Wind velocity resolved in world frame" annotation(
      Placement(transformation(origin = {220, 50}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {220, 50}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput airDensity(
    quantity = "Density",
    unit = "kg/m3") "Air density used by aerodynamic models" annotation(
      Placement(transformation(origin = {220, 10}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {220, 10}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput airTemperature(
    quantity = "ThermodynamicTemperature",
    unit = "K") "Air temperature signal" annotation(
      Placement(transformation(origin = {220, -30}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {220, -30}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput relativeHumidity(
    unit = "1") "Relative humidity signal" annotation(
      Placement(transformation(origin = {220, -70}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {220, -70}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput pressure(
    quantity = "Pressure",
    unit = "Pa") "Ambient pressure signal" annotation(
      Placement(transformation(origin = {220, -110}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {220, -110}, extent = {{-20, -20}, {20, 20}})));

protected
  parameter SI.Density rho = ambientPressure/(R*T);

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
  windVelocityWorld = v;
  airDensity = rho;
  airTemperature = T;
  relativeHumidity = h;
  pressure = ambientPressure;

  annotation(
    defaultComponentName = "atmosphere",
    defaultComponentPrefixes = "inner",
    Documentation(info = "<html>
<p>
Model <code>ConstantAtmosphere</code> is a BobLib constant-atmosphere adapter.
It preserves the VehicleInterfaces atmosphere function contract and exposes
wind velocity, density, temperature, humidity, and pressure as explicit signal
outputs for vehicle diagrams.
</p>
<p>
The standard vehicle template wires <code>airDensity</code> directly into the
aero subsystem and uses <code>windVelocityWorld</code> when computing relative
airspeed at the chassis CG.
</p>
</html>"));
end ConstantAtmosphere;
