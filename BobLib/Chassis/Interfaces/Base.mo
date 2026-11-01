within BobLib.Chassis.Interfaces;
partial model Base "Basic interface definition for a chassis"

  // Adapted from VehicleInterfaces 2.0.2: typed BobLib control bus.
  // Copyright (C) 2005-2013 Dassault Systemes, DLR and Modelon;
  // (C) 2013-2025 Modelica Association. Modelica License 1.1:
  // Resources/Licenses/LICENSE_VehicleInterfaces.txt.

  parameter Boolean usingMultiBodyDriveline = false
    "= true, if connecting to a MultiBody driveline"
    annotation(choices(checkBox = true), Dialog(tab = "Advanced", group = "Only for 1D chassis", enable = not includeWheelBearings));

protected
  parameter Boolean includeWheelBearings = false "Include wheel bearing connectors";
  parameter Boolean includeChassisFrame = false "Include the chassis frame";
  parameter Boolean includeSteeringWheel = false "Include the steering wheel connection";

public
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame

    if includeChassisFrame "Chassis reference frame (optional)"
    annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(extent = {{-176, -86}, {-144, -54}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a steeringWheel

    if includeSteeringWheel "Steering wheel connection (optional)"
    annotation(Placement(transformation(origin = {-160, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-10, 90}, {10, 110}})));

  BobLib.Utilities.Interfaces.ControlBus controlBus "Control signal bus"
    annotation(Placement(transformation(
        origin = {0, 100},
        extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-158, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_1(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    "Front left wheel"
    annotation(Placement(transformation(
        origin = {-90, -100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_2(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    "Front right wheel"
    annotation(Placement(transformation(
        origin = {-90, 100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_3(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    "Rear left wheel"
    annotation(Placement(transformation(
        origin = {90, -100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_4(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    "Rear right wheel"
    annotation(Placement(transformation(
        origin = {90, 100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_1(
    includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    annotation(Placement(transformation(extent = {{-118, -108}, {-106, -92}})));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_2(
    includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    annotation(Placement(transformation(extent = {{-118, 92}, {-106, 108}})));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_3(
    includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    annotation(Placement(transformation(extent = {{62, -108}, {74, -92}})));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_4(
    includeBearingConnector = includeWheelBearings or usingMultiBodyDriveline)
    annotation(Placement(transformation(extent = {{62, 92}, {74, 108}})));

equation
  connect(end_1.flange, wheelHub_1) annotation(Line(
      points = {{-110, -100}, {-90, -100}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_2.flange, wheelHub_2) annotation(Line(
      points = {{-110, 100}, {-90, 100}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_3.flange, wheelHub_3) annotation(Line(
      points = {{70, -100}, {90, -100}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_4.flange, wheelHub_4) annotation(Line(
      points = {{70, 100}, {90, 100}},
      color = {135, 135, 135},
      thickness = 0.5));
  annotation(
    Diagram(coordinateSystem(
        preserveAspectRatio = false,
        extent = {{-160, -100}, {160, 100}},
        grid = {1, 1}), graphics = {
        Rectangle(
          extent = {{-119, 108}, {-105, 92}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot),
        Rectangle(
          extent = {{61, 108}, {75, 92}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot),
        Rectangle(
          extent = {{61, -92}, {75, -108}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot),
        Rectangle(
          extent = {{-119, -92}, {-105, -108}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot)}),
    Icon(coordinateSystem(
        preserveAspectRatio = false,
        extent = {{-160, -100}, {160, 100}},
        grid = {1, 1})),
    Documentation(info = "<html>
<p>
This partial model defines the interfaces required for the chassis subsystem
of a&nbsp;two axled vehicle within the VehicleInterfaces package. See the
<a href=\"modelica://VehicleInterfaces.Chassis\">documentation</a> and
<a href=\"modelica://VehicleInterfaces.Chassis.Tutorial\">tutorial</a>
for more information.
</p>
</html>"));
end Base;
