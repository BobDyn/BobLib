within BobLibVehicleInterfaces.Chassis.Brakes;
model BasicVCUBrakes

  "Basic VCU-commanded mechanical brakes opposing wheel angular velocity"
  extends VehicleInterfaces.Icons.Brakes;
  extends VehicleInterfaces.Brakes.Interfaces.TwoAxleBase(
    final includeWheelBearings = false);

  import SI = Modelica.Units.SI;

  parameter SI.Torque maxTorque = 1500
    "Maximum combined mechanical brake torque magnitude";
  parameter Real frontBrakeBias(unit = "1", min = 0, max = 1) = 0.55
    "Mechanical bias-bar fraction assigned to the front hydraulic circuit";
  parameter SI.AngularVelocity wRegularization = 0.1
    "Wheel-speed magnitude where braking torque reaches full sign";

  output SI.Torque mechanicalBrakeTorqueRequest
    "Limited combined mechanical brake torque request";
  output SI.Torque frontCircuitBrakeTorqueRequest
    "Front hydraulic circuit brake torque request";
  output SI.Torque rearCircuitBrakeTorqueRequest
    "Rear hydraulic circuit brake torque request";
  output SI.Torque brakeTorque_1
    "Front-left brake torque magnitude";
  output SI.Torque brakeTorque_2
    "Front-right brake torque magnitude";
  output SI.Torque brakeTorque_3
    "Rear-left brake torque magnitude";
  output SI.Torque brakeTorque_4
    "Rear-right brake torque magnitude";

protected
  VehicleInterfaces.Interfaces.BrakesControlBus brakesControlBus
    "Brake-controller command bus tap" annotation(
      Placement(transformation(extent = {{-90, 36}, {-70, 56}})));
  VehicleInterfaces.Interfaces.BrakesBus brakesBus
    "Brake measurement bus tap" annotation(
      Placement(transformation(extent = {{-90, 10}, {-70, 30}})));

  Modelica.Blocks.Interfaces.RealInput mechanicalBrakeTorqueRequestBusTap(
    quantity = "Torque",
    unit = "N.m")
    "Combined mechanical brake torque request from brakesControlBus" annotation(
      Placement(transformation(origin = {-58, 46}, extent = {{-6, -6}, {6, 6}})));

  Modelica.Blocks.Sources.RealExpression wheelSpeed_1BusSignal(
    y = wheelSpeed_1.w) "Front-left wheel speed published to brakesBus" annotation(
      Placement(transformation(origin = {-52, 20}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression wheelSpeed_2BusSignal(
    y = wheelSpeed_2.w) "Front-right wheel speed published to brakesBus" annotation(
      Placement(transformation(origin = {-52, 10}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression wheelSpeed_3BusSignal(
    y = wheelSpeed_3.w) "Rear-left wheel speed published to brakesBus" annotation(
      Placement(transformation(origin = {-52, 0}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression wheelSpeed_4BusSignal(
    y = wheelSpeed_4.w) "Rear-right wheel speed published to brakesBus" annotation(
      Placement(transformation(origin = {-52, -10}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_1 annotation(
    Placement(transformation(origin = {-44, -72}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_2 annotation(
    Placement(transformation(origin = {-44, 72}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_3 annotation(
    Placement(transformation(origin = {44, -72}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_4 annotation(
    Placement(transformation(origin = {44, 72}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));

  Modelica.Mechanics.Rotational.Sources.Torque brakeTorqueSource_1 annotation(
    Placement(transformation(origin = {-24, -72}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Rotational.Sources.Torque brakeTorqueSource_2 annotation(
    Placement(transformation(origin = {-24, 72}, extent = {{-6, -6}, {6, 6}})));
  Modelica.Mechanics.Rotational.Sources.Torque brakeTorqueSource_3 annotation(
    Placement(transformation(origin = {24, -72}, extent = {{6, -6}, {-6, 6}})));
  Modelica.Mechanics.Rotational.Sources.Torque brakeTorqueSource_4 annotation(
    Placement(transformation(origin = {24, 72}, extent = {{6, -6}, {-6, 6}})));

  Real wheelDirection_1(unit = "1");
  Real wheelDirection_2(unit = "1");
  Real wheelDirection_3(unit = "1");
  Real wheelDirection_4(unit = "1");

equation
  mechanicalBrakeTorqueRequest = 
    noEvent(min(maxTorque, max(0, mechanicalBrakeTorqueRequestBusTap)));

  frontCircuitBrakeTorqueRequest = 
    frontBrakeBias*mechanicalBrakeTorqueRequest;
  rearCircuitBrakeTorqueRequest = 
    (1 - frontBrakeBias)*mechanicalBrakeTorqueRequest;

  brakeTorque_1 = frontCircuitBrakeTorqueRequest/2;
  brakeTorque_2 = frontCircuitBrakeTorqueRequest/2;
  brakeTorque_3 = rearCircuitBrakeTorqueRequest/2;
  brakeTorque_4 = rearCircuitBrakeTorqueRequest/2;

  wheelDirection_1 = 
    noEvent(
      if wheelSpeed_1.w > wRegularization then 1
      elseif wheelSpeed_1.w < -wRegularization then -1
      else wheelSpeed_1.w/max(wRegularization, 1e-6));
  wheelDirection_2 = 
    noEvent(
      if wheelSpeed_2.w > wRegularization then 1
      elseif wheelSpeed_2.w < -wRegularization then -1
      else wheelSpeed_2.w/max(wRegularization, 1e-6));
  wheelDirection_3 = 
    noEvent(
      if wheelSpeed_3.w > wRegularization then 1
      elseif wheelSpeed_3.w < -wRegularization then -1
      else wheelSpeed_3.w/max(wRegularization, 1e-6));
  wheelDirection_4 = 
    noEvent(
      if wheelSpeed_4.w > wRegularization then 1
      elseif wheelSpeed_4.w < -wRegularization then -1
      else wheelSpeed_4.w/max(wRegularization, 1e-6));

  brakeTorqueSource_1.tau = -brakeTorque_1*wheelDirection_1;
  brakeTorqueSource_2.tau = -brakeTorque_2*wheelDirection_2;
  brakeTorqueSource_3.tau = -brakeTorque_3*wheelDirection_3;
  brakeTorqueSource_4.tau = -brakeTorque_4*wheelDirection_4;

  connect(controlBus.brakesControlBus, brakesControlBus) annotation(
    Line(points = {{-100, 60}, {-80, 60}, {-80, 46}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.brakesBus, brakesBus) annotation(
    Line(points = {{-100, 60}, {-86, 60}, {-86, 20}, {-80, 20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(brakesControlBus.mechanicalBrakeTorqueRequest, mechanicalBrakeTorqueRequestBusTap) annotation(
    Line(points = {{-80, 46}, {-60, 46}}, color = {0, 0, 127}));

  connect(wheelSpeed_1.flange, wheelHub_1.flange) annotation(
    Line(points = {{-44, -78}, {-44, -100}, {-60, -100}}));
  connect(wheelSpeed_2.flange, wheelHub_2.flange) annotation(
    Line(points = {{-44, 78}, {-44, 100}, {-60, 100}}));
  connect(wheelSpeed_3.flange, wheelHub_3.flange) annotation(
    Line(points = {{44, -78}, {44, -100}, {60, -100}}));
  connect(wheelSpeed_4.flange, wheelHub_4.flange) annotation(
    Line(points = {{44, 78}, {44, 100}, {60, 100}}));

  connect(brakeTorqueSource_1.flange, wheelHub_1.flange) annotation(
    Line(points = {{-18, -72}, {-18, -100}, {-60, -100}}));
  connect(brakeTorqueSource_2.flange, wheelHub_2.flange) annotation(
    Line(points = {{-18, 72}, {-18, 100}, {-60, 100}}));
  connect(brakeTorqueSource_3.flange, wheelHub_3.flange) annotation(
    Line(points = {{18, -72}, {18, -100}, {60, -100}}));
  connect(brakeTorqueSource_4.flange, wheelHub_4.flange) annotation(
    Line(points = {{18, 72}, {18, 100}, {60, 100}}));

  connect(wheelSpeed_1BusSignal.y, brakesBus.wheelSpeed_1) annotation(
    Line(points = {{-43.2, 20}, {-80, 20}}, color = {0, 0, 127}));
  connect(wheelSpeed_2BusSignal.y, brakesBus.wheelSpeed_2) annotation(
    Line(points = {{-43.2, 10}, {-80, 10}, {-80, 20}}, color = {0, 0, 127}));
  connect(wheelSpeed_3BusSignal.y, brakesBus.wheelSpeed_3) annotation(
    Line(points = {{-43.2, 0}, {-82, 0}, {-82, 20}, {-80, 20}}, color = {0, 0, 127}));
  connect(wheelSpeed_4BusSignal.y, brakesBus.wheelSpeed_4) annotation(
    Line(points = {{-43.2, -10}, {-84, -10}, {-84, 20}, {-80, 20}}, color = {0, 0, 127}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>BasicVCUBrakes</code> is a minimal BobLib brake model exposed
through the standard VehicleInterfaces two-axle brakes boundary. It subscribes
to <code>controlBus.brakesControlBus.mechanicalBrakeTorqueRequest</code>,
limits that positive combined brake torque request, splits it across the
front and rear hydraulic circuits using a mechanical bias-bar ratio, then
applies torque opposite each wheel's rotational velocity.
</p>
<p>
This model intentionally does not implement hydraulic dynamics, ABS, thermal
behavior, or static holding torque. It is a simple VCU-commanded mechanical
brake actuator for early vehicle-level studies. The bias ratio is kept as the
obvious insertion point for future pedal-box modeling, bias-bar compliance, and
hydraulic damping.
</p>
</html>"));
end BasicVCUBrakes;
