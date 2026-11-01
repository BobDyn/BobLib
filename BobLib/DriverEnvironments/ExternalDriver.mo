within BobLib.DriverEnvironments;
model ExternalDriver "VI normalized driver interface with BobLib steering actuation"

  extends VehicleInterfaces.Icons.DriverEnvironment;

  extends BobLib.DriverEnvironments.Interfaces.Base(
    final includeDriverSeat = false, final includeSteeringWheel = true,
    final includeAcceleratorPedal = false, final includeBrakePedal = usePhysicalBrakePedal);
  replaceable VehicleInterfaces.Interfaces.DriverInterface driverInterface
    "Driver Interaction Bus" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-100, 0})));

  parameter Boolean usePhysicalBrakePedal = false "Expose physical pedal travel to the brake subsystem instead of bus-only brake demand" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean includeManeuverMeasurements = false "Publish the vehicle measurements required by the standard maneuver driver" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean exactSteering = true "Prescribe steering angle exactly; otherwise use the steering input filter" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Modelica.Units.SI.Length pedalFullTravel = 0.05 "Pedal displacement corresponding to a normalized brake demand of one" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Frequency pedalFilterFrequency = 50 "Physical pedal actuator bandwidth" annotation(Dialog(group = "Operating settings"));
  parameter Real steeringCenter(unit = "1") = 0.5 "Normalized straight-ahead position" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Angle steeringRange = 4*Modelica.Constants.pi
    "Handwheel angle change from normalized zero to one" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Frequency steeringFilterFrequency = 50 "Cutoff frequency of the steering position filter when exactSteering is false" annotation(Dialog(group = "Operating settings"));

  BobLib.DriverEnvironments.CommandedDriveByWire actuation(
    exactSteering = exactSteering, steeringFilterFrequency = steeringFilterFrequency)
    annotation(Placement(transformation(origin = {65, 0}, extent = {{-15, -15}, {15, 15}})));
  VehicleInterfaces.Mechanics.NormalisedRotational.AngleSensor steering
    annotation(Placement(transformation(origin = {-60, 50}, extent = {{-5, -5}, {5, 5}})));

  Modelica.Blocks.Sources.RealExpression handwheelAngle(y = steeringRange*(steering.phi-steeringCenter))
    annotation(Placement(transformation(origin = {9.35, 50}, extent = {{-4, -4}, {4, 4}})));
  VehicleInterfaces.Mechanics.NormalisedTranslational.PositionSensor accelerator
    annotation(Placement(transformation(origin = {-60, 30}, extent = {{-5, -5}, {5, 5}})));
  VehicleInterfaces.Mechanics.NormalisedTranslational.PositionSensor brake
    annotation(Placement(transformation(origin = {-60, 10}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Routing.IntegerPassThrough gearboxMode
    annotation(Placement(transformation(origin = {-40, -20}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Routing.IntegerPassThrough requestedGear
    annotation(Placement(transformation(origin = {-40, -38}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Routing.IntegerPassThrough ignition
    annotation(Placement(transformation(origin = {-40, -56}, extent = {{-5, -5}, {5, 5}})));
  Modelica.Blocks.Routing.RealPassThrough vehicleSpeed
    annotation(Placement(transformation(origin = {20, -65}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
  Modelica.Blocks.Routing.RealPassThrough motorSpeed
    annotation(Placement(transformation(origin = {20, -85}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
  BobLib.DriverEnvironments.Internal.ManeuverMeasurements maneuverMeasurements if includeManeuverMeasurements
    annotation(Placement(visible = includeManeuverMeasurements, transformation(origin = {30, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));

  Modelica.Mechanics.Translational.Sources.Position pedalMotion(
    exact = false,
    f_crit = pedalFilterFrequency,
    s_ref = pedalFullTravel*brake.position) if usePhysicalBrakePedal
    annotation(Placement(visible = usePhysicalBrakePedal, transformation(origin = {65, -90}, extent = {{-5, -5}, {5, 5}})));

protected
  BobLib.DriverEnvironments.Interfaces.DriverBus driverBus
    "Typed driverBus signal contract" annotation(
      Placement(transformation(origin = {0, -38}, extent = {{-15, -15}, {15, 15}})));
  BobLib.Chassis.Interfaces.ChassisBus chassisBus
    "Typed chassisBus signal contract" annotation(
      Placement(transformation(origin = {80, -65}, extent = {{-5, -5}, {5, 5}})));
  BobLib.ElectricDrives.Interfaces.ElectricMotorBus electricMotorBus
    "Typed electricMotorBus signal contract" annotation(
      Placement(transformation(origin = {80, -85}, extent = {{-5, -5}, {5, 5}})));

equation
  connect(driverBus, controlBus.driverBus) annotation(
    Line(points = {{0, -38}, {0, -20}, {94, -20}, {94, 60}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(chassisBus, controlBus.chassisBus) annotation(
    Line(points = {{80, -65}, {94, -65}, {94, 60}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(electricMotorBus, controlBus.electricMotorBus) annotation(
    Line(points = {{80, -85}, {94, -85}, {94, 60}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(
    pedalMotion.flange,
    brakePedal) annotation(Line(visible = usePhysicalBrakePedal, points = {{70, -90}, {85, -90}, {85, -96}, {40, -96}, {40, -100}}, color = {0, 127, 0}));
  connect(
    controlBus.chassisBus,
    maneuverMeasurements.chassisBus) annotation(Line(visible = includeManeuverMeasurements, points = {{100, 60}, {94, 60}, {94, -40}, {36, -40}}, color = {255, 204, 51}));
  connect(
    maneuverMeasurements.driverInterface,
    driverInterface) annotation(Line(visible = includeManeuverMeasurements, points = {{24, -40}, {20, -40}, {20, -60}, {-98, -60}, {-98, 0}, {-100, 0}}, color = {85, 85, 255}));
  assert(not usePhysicalBrakePedal or (pedalFullTravel>0 and pedalFilterFrequency>0),
    "Physical pedal requires positive travel and bandwidth");
  assert(steeringRange>0, "ExternalDriver requires a positive steering range");
  connect(driverInterface.steeringWheel, steering.flange_a) annotation(Line(points = {{-100, 0}, {-90, 0}, {-90, 50}, {-65, 50}}, color = {95, 95, 95}));
  connect(driverInterface.acceleratorPedal, accelerator.flange_a) annotation(Line(points = {{-100, 0}, {-85, 0}, {-85, 30}, {-65, 30}}, color = {0, 127, 0}));
  connect(driverInterface.brakePedal, brake.flange_a) annotation(Line(points = {{-100, 0}, {-80, 0}, {-80, 10}, {-65, 10}}, color = {0, 127, 0}));
  connect(handwheelAngle.y, actuation.steeringAngleCommand) annotation(Line(points = {{13.75, 50}, {35, 50}, {35, 12}, {48.5, 12}}, color = {0, 0, 127}));
  connect(accelerator.position, actuation.acceleratorPedalCommand) annotation(Line(points = {{-54.5, 30}, {30, 30}, {30, 7.5}, {48.5, 7.5}}, color = {0, 0, 127}));
  connect(brake.position, actuation.brakePedalCommand) annotation(Line(points = {{-54.5, 10}, {25, 10}, {25, 3}, {48.5, 3}}, color = {0, 0, 127}));
  connect(actuation.controlBus, controlBus) annotation(Line(points = {{80, 9}, {84, 9}, {84, 52}, {94, 52}, {94, 60}, {100, 60}}, color = {255, 204, 51}));
  connect(actuation.steeringWheel, steeringWheel) annotation(Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(driverInterface.gearboxMode, gearboxMode.u) annotation(Line(points = {{-100, 0}, {-85, 0}, {-85, -20}, {-46, -20}}, color = {255, 127, 0}));
  connect(gearboxMode.y, driverBus.gearboxMode) annotation(Line(points = {{-34.5, -20}, {-25, -20}, {-25, -30}, {0, -30}, {0, -38}}, color = {255, 127, 0}));
  connect(driverInterface.requestedGear, requestedGear.u) annotation(Line(points = {{-100, 0}, {-90, 0}, {-90, -38}, {-46, -38}}, color = {255, 127, 0}));
  connect(requestedGear.y, driverBus.requestedGear) annotation(Line(points = {{-34.5, -38}, {0, -38}}, color = {255, 127, 0}));
  connect(driverInterface.ignition, ignition.u) annotation(Line(points = {{-100, 0}, {-95, 0}, {-95, -56}, {-46, -56}}, color = {255, 127, 0}));
  connect(ignition.y, driverBus.ignition) annotation(Line(points = {{-34.5, -56}, {-25, -56}, {-25, -46}, {0, -46}, {0, -38}}, color = {255, 127, 0}));
  connect(chassisBus.vehicleSpeed, vehicleSpeed.u) annotation(Line(points = {{80, -65}, {26, -65}}, color = {0, 0, 127}));
  connect(vehicleSpeed.y, driverInterface.vehicleSpeed) annotation(Line(points = {{14.5, -65}, {-97, -65}, {-97, 0}, {-100, 0}}, color = {0, 0, 127}));
  connect(electricMotorBus.speed, motorSpeed.u) annotation(Line(points = {{80, -85}, {26, -85}}, color = {0, 0, 127}));
  connect(motorSpeed.y, driverInterface.motorSpeed) annotation(Line(points = {{14.5, -85}, {-99, -85}, {-99, 0}, {-100, 0}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 80}}, preserveAspectRatio = true)),
    Documentation(info = "<html><p>Connect a VI driver to driverInterface. Normalized steering maps to
<code>steeringRange*(phi-steeringCenter)</code> radians; pedals pass through as normalized intent.
The same CommandedDriveByWire actuator used by scalar-command vehicles owns steering and pedal
publication. This adapter publishes gearboxMode, requestedGear and ignition and returns vehicleSpeed
and motorSpeed. EV enable remains the vehicle supervisor's responsibility.</p>
<p>This is a drive-by-wire interface: normalized controls are sensed without force feedback.
It does not claim a physical pedal linkage or a haptic steering system. A VI MinimalDriver needs
its steeringAngle.k set to steeringCenter for straight-ahead operation.</p></html>"));
end ExternalDriver;
