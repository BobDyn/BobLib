within BobLib.Brakes.Interfaces;
partial model Base "Base interface definition for a brakes"

  // Adapted from VehicleInterfaces 2.0.2: typed BobLib control bus.
  // Copyright (C) 2005-2013 Dassault Systemes, DLR and Modelon;
  // (C) 2013-2025 Modelica Association. Modelica License 1.1:
  // Resources/Licenses/LICENSE_VehicleInterfaces.txt.

  parameter Boolean usingMultiBodyChassis = false
    "= true, if connecting to a MultiBody chassis"
    annotation(choices(checkBox = true), Dialog(tab = "Advanced", group = "Only for 1D brakes", enable = not includeWheelBearings));

protected
  parameter Boolean includeBrakePedal = false "Include the brake pedal connection";
  parameter Boolean includeWheelBearings = false "Include the wheel bearing connectors";

public
  Modelica.Mechanics.Translational.Interfaces.Flange_a brakePedal

    if includeBrakePedal "Brake pedal connection (optional)"
    annotation(Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-10, 110}, {10, 90}})));
  BobLib.Utilities.Interfaces.ControlBus controlBus "Control signal bus"
    annotation(Placement(transformation(origin = {-160, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(
        origin = {-100, 60},
        extent = {{-20, -20}, {20, 20}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_1(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    "Front left wheel"
    annotation(Placement(transformation(origin = {-160, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270), iconTransformation(
        origin = {-60, -100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_2(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    "Front right wheel"
    annotation(Placement(transformation(origin = {160, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(
        origin = {-60, 100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_3(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    "Rear left wheel"
    annotation(Placement(transformation(origin = {-160, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 270), iconTransformation(
        origin = {60, -100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_4(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    "Rear right wheel"
    annotation(Placement(transformation(origin = {160, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(
        origin = {60, 100},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_1(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    annotation(Placement(transformation(origin = {-140, 60}, extent = {{-6, -8}, {6, 8}}, rotation = 0)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_2(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    annotation(Placement(transformation(origin = {140, 60}, extent = {{-6, -8}, {6, 8}}, rotation = 0)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_3(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    annotation(Placement(transformation(origin = {-140, -60}, extent = {{-6, -8}, {6, 8}}, rotation = 0)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_4(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    annotation(Placement(transformation(origin = {140, -60}, extent = {{-6, -8}, {6, 8}}, rotation = 0)));

  parameter Boolean useDifferentialInputBrake = false "Replace rear wheel brakes with one differential-input brake" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing differentialInput(
    includeBearingConnector = includeWheelBearings or usingMultiBodyChassis) if useDifferentialInputBrake annotation(
    Placement(transformation(origin = {160, -80}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(end_1.flange, wheelHub_1) annotation(Line(
      points = {{-138, 60}, {-160, 60}, {-160, 40}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_2.flange, wheelHub_2) annotation(Line(
      points = {{142, 60}, {160, 60}, {160, 40}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_3.flange, wheelHub_3) annotation(Line(
      points = {{-138, -60}, {-160, -60}, {-160, -40}},
      color = {135, 135, 135},
      thickness = 0.5));
  connect(end_4.flange, wheelHub_4) annotation(Line(
      points = {{142, -60}, {160, -60}, {160, -40}},
      color = {135, 135, 135},
      thickness = 0.5));
  annotation(Documentation(info = "<html>
<p>
This partial model defines the interfaces required for the brakes subsystem
of a two axled vehicle within the VehicleInterfaces package. See the
<a href=\"modelica://VehicleInterfaces.Brakes\">documentation</a> and
<a href=\"modelica://VehicleInterfaces.Brakes.Tutorial\">tutorial</a>
for more information.
</p>
</html>"),
       Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-160, -100}, {160, 100}})));
end Base;
