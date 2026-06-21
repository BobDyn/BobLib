within BobLib.Aero.Interfaces;

partial model Base

  "Basic interface definition for an aerodynamic subsystem"
  import SI = Modelica.Units.SI;

  parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));
  parameter SI.Position mountOffset[3] = {0, 0, 0}
    "Offset from sprung chassis frame to aero load frame";

  VehicleInterfaces.Interfaces.ControlBus controlBus
    "VehicleInterfaces control bus carrying chassis measurements" annotation(
      Placement(transformation(origin = {-160, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-120, 100}, extent = {{-20, -20}, {20, 20}})));
  BobLib.Atmospheres.Interfaces.AtmosphereBus atmosphereBus
    "Atmosphere signal bus carrying atmosphere-owned measurements" annotation(
      Placement(transformation(origin = {-160, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-60, 100}, extent = {{-20, -20}, {20, 20}})));

  output SI.Force force[3]
    "Aerodynamic force resolved in aero load frame";
  output SI.Torque torque[3]
    "Aerodynamic torque resolved in aero load frame";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungChassisFrame
    "Sprung chassis frame that carries the aerodynamic loads" annotation(
      Placement(transformation(extent = {{-176, -16}, {-144, 16}})));

protected
  import Modelica.Mechanics.MultiBody.Frames;

  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    "VehicleInterfaces chassis signal bus tap" annotation(
      Placement(
        transformation(origin = {20, 28}, extent = {{-120, 42}, {-100, 62}}),
        iconTransformation(extent = {{-120, 42}, {-100, 62}})));
  SI.Length rideHeight_1
    "Front-left ride height used by the aerodynamic model";
  SI.Length rideHeight_2
    "Front-right ride height used by the aerodynamic model";
  SI.Length rideHeight_3
    "Rear-left ride height used by the aerodynamic model";
  SI.Length rideHeight_4
    "Rear-right ride height used by the aerodynamic model";
  SI.Density airDensity
    "Local air density from atmosphereBus";
  SI.Temperature airTemperature
    "Local air temperature from atmosphereBus";
  Real relativeHumidity(unit = "1")
    "Local relative humidity from atmosphereBus";
  SI.AbsolutePressure airPressure
    "Local ambient pressure from atmosphereBus";
  SI.Velocity windVelocityWorld[3]
    "Wind velocity resolved in world frame from atmosphereBus";
  SI.Velocity bodyVelocity[3]
    "Sprung chassis velocity resolved in chassis frame";
  SI.Velocity windVelocityBody[3]
    "Wind velocity resolved in chassis frame";
  SI.Velocity relativeAirVelocity[3]
    "Vehicle velocity relative to local air, resolved in chassis frame";
  SI.Velocity relativeAirSpeed
    "Vehicle speed relative to local air";

  Modelica.Blocks.Interfaces.RealInput rideHeight_1BusTap(
    quantity = "Length",
    unit = "m") "Front-left ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {0, 80}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 30}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_2BusTap(
    quantity = "Length",
    unit = "m") "Front-right ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {0, 68}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 20}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_3BusTap(
    quantity = "Length",
    unit = "m") "Rear-left ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {0, 56}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 10}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput rideHeight_4BusTap(
    quantity = "Length",
    unit = "m") "Rear-right ride-height tap from chassisBus" annotation(
      Placement(
        transformation(origin = {0, 44}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, 0}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput airDensityBusTap(
    quantity = "Density",
    unit = "kg/m3") "Air-density tap from atmosphereBus" annotation(
      Placement(
        transformation(origin = {0, -48}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, -18}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput windVelocityWorldBusTap[3](
    each quantity = "Velocity",
    each unit = "m/s") "World-frame wind-velocity tap from atmosphereBus" annotation(
      Placement(
        transformation(origin = {0, -36}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, -34}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput airTemperatureBusTap(
    quantity = "ThermodynamicTemperature",
    unit = "K") "Air-temperature tap from atmosphereBus" annotation(
      Placement(
        transformation(origin = {0, -24}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, -50}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput relativeHumidityBusTap(unit = "1")
    "Relative-humidity tap from atmosphereBus" annotation(
      Placement(
        transformation(origin = {0, -12}, extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, -66}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput pressureBusTap(
    quantity = "Pressure",
    unit = "Pa") "Pressure tap from atmosphereBus" annotation(
      Placement(
        transformation(extent = {{-8, -8}, {8, 8}}),
        iconTransformation(origin = {-100, -82}, extent = {{-8, -8}, {8, 8}})));

  BobLib.Aero.Mounts.RigidMount rigidMount(
    r = mountOffset,
    animation = false) annotation(
      Placement(transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque aeroLoads(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b,
    animation = not headless) annotation(
      Placement(transformation(origin = {130, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

equation
  aeroLoads.force = force;
  aeroLoads.torque = torque;
  rideHeight_1 = rideHeight_1BusTap;
  rideHeight_2 = rideHeight_2BusTap;
  rideHeight_3 = rideHeight_3BusTap;
  rideHeight_4 = rideHeight_4BusTap;
  airDensity = airDensityBusTap;
  windVelocityWorld = windVelocityWorldBusTap;
  airTemperature = airTemperatureBusTap;
  relativeHumidity = relativeHumidityBusTap;
  airPressure = pressureBusTap;
  bodyVelocity =
    Frames.resolve2(sprungChassisFrame.R, der(sprungChassisFrame.r_0));
  windVelocityBody =
    Frames.resolve2(sprungChassisFrame.R, windVelocityWorld);
  relativeAirVelocity =
    bodyVelocity - windVelocityBody;
  relativeAirSpeed =
    sqrt(
      relativeAirVelocity[1]^2 +
      relativeAirVelocity[2]^2 +
      relativeAirVelocity[3]^2);

  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{-160, 80}, {-90, 80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(atmosphereBus.airDensity, airDensityBusTap) annotation(
    Line(points = {{-160, 40}, {-89, 40}, {-89, -48}, {0, -48}}, color = {0, 0, 127}));
  connect(atmosphereBus.windVelocityWorld, windVelocityWorldBusTap) annotation(
    Line(points = {{-160, 40}, {-89, 40}, {-89, -36}, {0, -36}}, color = {0, 0, 127}));
  connect(atmosphereBus.airTemperature, airTemperatureBusTap) annotation(
    Line(points = {{-160, 40}, {-89, 40}, {-89, -24}, {0, -24}}, color = {0, 0, 127}));
  connect(atmosphereBus.relativeHumidity, relativeHumidityBusTap) annotation(
    Line(points = {{-160, 40}, {-89, 40}, {-89, -12}, {0, -12}}, color = {0, 0, 127}));
  connect(atmosphereBus.pressure, pressureBusTap) annotation(
    Line(points = {{-160, 40}, {-89, 40}, {-89, 0}, {0, 0}}, color = {0, 0, 127}));

  connect(sprungChassisFrame, rigidMount.sprungChassisFrame) annotation(
    Line(points = {{-160, 0}, {-120, 0}, {-120, -70}, {-78, -70}}, color = {95, 95, 95}));
  connect(rigidMount.aeroFrame, aeroLoads.frame_b) annotation(
    Line(points = {{-62, -70}, {100, -70}, {100, 0}, {120, 0}}, color = {95, 95, 95}));
  connect(chassisBus.rideHeight_1, rideHeight_1BusTap) annotation(
    Line(points = {{-90, 80}, {0, 80}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_2, rideHeight_2BusTap) annotation(
    Line(points = {{-90, 80}, {-30, 80}, {-30, 68}, {0, 68}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_3, rideHeight_3BusTap) annotation(
    Line(points = {{-90, 80}, {-30, 80}, {-30, 56}, {0, 56}}, color = {0, 0, 127}));
  connect(chassisBus.rideHeight_4, rideHeight_4BusTap) annotation(
    Line(points = {{-90, 80}, {-30, 80}, {-30, 44}, {0, 44}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}})),
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100}, {160, 100}}), graphics = {
      Rectangle(extent = {{-140, 60}, {140, -60}}, lineColor = {95, 95, 95}, fillColor = {245, 245, 245}, fillPattern = FillPattern.Solid, radius = 8),
      Line(points = {{-120, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 1),
      Polygon(points = {{-25, 0}, {30, 30}, {95, 30}, {70, 0}, {95, -30}, {30, -30}, {-25, 0}}, lineColor = {0, 85, 170}, fillColor = {210, 230, 255}, fillPattern = FillPattern.Solid),
      Text(extent = {{-80, 90}, {80, 60}}, textString = "Aero")}),
    Documentation(info = "<html>
<p>
This partial model defines the basic interfaces required for an aerodynamic
subsystem. Implementations provide <code>force</code> and <code>torque</code>
from chassis-owned ride heights, atmosphere-owned density and wind, and the
sprung chassis frame velocity. Ride heights are read from
<code>controlBus.chassisBus</code>; atmosphere signals are read from the shared
<code>atmosphereBus</code>. The base interface applies those loads to
<code>sprungChassisFrame</code> through a rigid aero mount.
</p>
</html>"));
end Base;