within BobLib.Experiments.Standards;
model ManeuverFMI "Standard driver policy with scalar FMI measurements and commands"

  extends BobLib.Icons.SimulationIcon;
  parameter Integer mode = 0 "Standard driver maneuver selector; uses the mode codes documented by Drivers.Internal.Maneuver" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Torque tau_max = 240 "Motor torque capacity used to normalize accelerator demand after final-drive multiplication" annotation(Evaluate = false, Dialog(group = "Operating settings"));
  parameter Real finalDriveRatio = 1 "Motor-to-wheel speed ratio used to convert motor torque capacity to wheel torque" annotation(Evaluate = false, Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Torque mechanicalBrakeTorqueLimit = 1500 "Mechanical wheel-braking torque capacity used to normalize brake demand" annotation(Evaluate = false, Dialog(group = "Operating settings"));

  Modelica.Blocks.Interfaces.RealInput speed(unit = "m/s") annotation(Placement(transformation(origin = {-100, 80}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput ay(unit = "m/s2") annotation(Placement(transformation(origin = {-100, 40}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Blocks.Interfaces.RealInput loads[4](each unit = "N") annotation(Placement(transformation(origin = {-100, -20}, extent = {{-8, -8}, {8, 8}})));
  output Modelica.Units.SI.Angle steering = driver.steeringAngleCommand;
  output Real accelerator(unit = "1") = driver.acceleratorPedalCommand;
  output Real brake(unit = "1") = driver.brakePedalCommand;
  output Modelica.Units.SI.Acceleration ayTarget = driver.steadyStateAyCommand;
  output Boolean rampEnding = driver.rampEnding;
  output Boolean targetReached = driver.steadyStateTargetReached;

  BobLib.Drivers.StandardDriver driver(useMode = mode,
    driveTorqueCapacity = tau_max*finalDriveRatio, brakeTorqueCapacity = mechanicalBrakeTorqueLimit)
    annotation(Placement(transformation(origin = {0, 30}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(speed, driver.driverInterface.vehicleSpeed) annotation(Line(points = {{-100, 80}, {60, 80}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  connect(ay, driver.driverInterface.bodyAcceleration_2) annotation(Line(points = {{-100, 40}, {-60, 40}, {-60, 60}, {60, 60}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  connect(loads[1], driver.driverInterface.Fz_1) annotation(Line(points = {{-100, -20}, {60, -20}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  connect(loads[2], driver.driverInterface.Fz_2) annotation(Line(points = {{-100, -20}, {60, -20}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  connect(loads[3], driver.driverInterface.Fz_3) annotation(Line(points = {{-100, -20}, {60, -20}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  connect(loads[4], driver.driverInterface.Fz_4) annotation(Line(points = {{-100, -20}, {60, -20}, {60, 30}, {20, 30}}, color = {0, 0, 127}));
  annotation(experiment(StopTime = 10, Tolerance = 1e-6), Diagram(coordinateSystem(extent = {{-100, -80}, {100, 100}})),
    Documentation(info = "<html><p>Scalar wrapper for exactly the same StandardDriver policy used by VehicleSim. The plant receives steering angle in radians and normalized pedals. Inputs are vehicle speed, lateral acceleration, and four tire normal loads. Set mode before initialization. This controller FMU is separate from the standalone vehicle plant FMU.</p></html>"));
end ManeuverFMI;
