within BobLib.Drivelines.Interfaces;
partial model Base "Basic interface definition for a driveline"

  // Adapted from VehicleInterfaces 2.0.2: typed BobLib control bus.
  // Copyright (C) 2005-2013 Dassault Systemes, DLR and Modelon;
  // (C) 2013-2025 Modelica Association. Modelica License 1.1:
  // Resources/Licenses/LICENSE_VehicleInterfaces.txt.

  parameter Boolean usingMultiBodyChassis = false
    "= true, if connecting to a MultiBody chassis"
    annotation(choices(checkBox = true), Dialog(tab = "Advanced", group = "Only for 1D driveline", enable = not includeWheelBearings));
  parameter Boolean usingMultiBodyTransmission = false
    "= true, if connecting to a MultiBody transmission"
    annotation(choices(checkBox = true), Dialog(tab = "Advanced", group = "Only for 1D driveline", enable = not includeTransmissionBearing));

protected
  parameter Boolean includeWheelBearings = false "Include wheel bearing connectors";
  parameter Boolean includeMount = false "Include the driveline mount";
  parameter Boolean includeTransmissionBearing = false
    "Include the transmission bearing";

public
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chassisFrame
    if includeMount "Chassis attachment for the driveline's internal mount"
    annotation(Placement(transformation(origin = {-140, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90), iconTransformation(origin = {-80, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing transmissionFlange(
    final includeBearingConnector = includeTransmissionBearing or usingMultiBodyTransmission)
    "Connection to the Transmission output shaft"
    annotation(Placement(transformation(origin = {-160, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a drivelineMount

    if includeMount "Driveline mount connection (optional)"
    annotation(Placement(transformation(
        origin = {0, -100},
        extent = {{-16, -16}, {16, 16}},
        rotation = 90), iconTransformation(
        origin = {0, -100},
        extent = {{-16, -16}, {16, 16}},
        rotation = 90)));
  BobLib.Utilities.Interfaces.ControlBus controlBus "Control signal bus"
    annotation(Placement(transformation(
        origin = {-160, 60},
        extent = {{-10, -10}, {10, 10}},
        rotation = 90), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_0(
    final includeBearingConnector = includeTransmissionBearing or usingMultiBodyTransmission)
    annotation(Placement(transformation(
        origin = {-160, 22},
        extent = {{-6, -8}, {6, 8}},
        rotation = 270), iconTransformation(
        origin = {-100, 22},
        extent = {{-6, -8}, {6, 8}},
        rotation = 270)));

  parameter BobLib.Drivelines.Types.DriveLayout driveLayout = BobLib.Drivelines.Types.DriveLayout.Rear
    "Driven axles; optional undriven brake shafts are selected separately" annotation(Dialog(group = "Architecture"), Evaluate = true);

  parameter Boolean useFrontInboardBrakes = false "Expose front differential-side brake ports and their halfshaft reaction paths" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean useRearInboardBrakes = false "Expose rear differential-side brake ports and their halfshaft reaction paths" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_1(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    if driveLayout <> BobLib.Drivelines.Types.DriveLayout.Rear or useFrontInboardBrakes "Front left wheel shaft" annotation(
    Placement(
      transformation(origin = {-96, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90),
      iconTransformation(origin = {-60, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_2(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    if driveLayout <> BobLib.Drivelines.Types.DriveLayout.Rear or useFrontInboardBrakes "Front right wheel shaft" annotation(
    Placement(
      transformation(origin = {-96, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90),
      iconTransformation(origin = {-60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_3(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    if driveLayout <> BobLib.Drivelines.Types.DriveLayout.Front or useRearInboardBrakes "Rear left wheel shaft" annotation(
    Placement(
      transformation(origin = {96, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90),
      iconTransformation(origin = {60, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing wheelHub_4(
    final includeBearingConnector = includeWheelBearings or usingMultiBodyChassis)
    if driveLayout <> BobLib.Drivelines.Types.DriveLayout.Front or useRearInboardBrakes "Rear right wheel shaft" annotation(
    Placement(
      transformation(origin = {96, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90),
      iconTransformation(origin = {60, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing inboardBrake_1(
    final includeBearingConnector = usingMultiBodyTransmission) if useFrontInboardBrakes
    "Inboard brake rotor and sprung caliper support" annotation(
    Placement(
      transformation(origin = {-40, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90),
      iconTransformation(origin = {-25, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing inboardBrake_2(
    final includeBearingConnector = usingMultiBodyTransmission) if useFrontInboardBrakes
    "Inboard brake rotor and sprung caliper support" annotation(
    Placement(
      transformation(origin = {-160, 80}, extent = {{-8, -8}, {8, 8}}, rotation = 90),
      iconTransformation(origin = {-25, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing inboardBrake_3(
    final includeBearingConnector = usingMultiBodyTransmission) if useRearInboardBrakes
    "Inboard brake rotor and sprung caliper support" annotation(
    Placement(
      transformation(origin = {160, -30}, extent = {{-8, -8}, {8, 8}}, rotation = 90),
      iconTransformation(origin = {25, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing inboardBrake_4(
    final includeBearingConnector = usingMultiBodyTransmission) if useRearInboardBrakes
    "Inboard brake rotor and sprung caliper support" annotation(
    Placement(
      transformation(origin = {40, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90),
      iconTransformation(origin = {25, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  parameter Boolean useDifferentialInputBrake = false
    "Expose the brake shaft upstream of the final-drive reduction"
    annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing differentialBrakeShaft(
    includeBearingConnector = usingMultiBodyTransmission)
    if useDifferentialInputBrake
    "Brake shaft and optional housing bearing frame upstream of final drive"
    annotation(Placement(transformation(origin = {-160, -60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(end_0.flange, transmissionFlange) annotation(Line(
      points = {{-160, 20}, {-160, 0}},
      color = {135, 135, 135},
      thickness = 0.5));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-160, -100},
            {160, 100}}), graphics = {Rectangle(
          extent = {{-12, -88}, {12, -112}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot), Rectangle(
          extent = {{-168, 28}, {-152, 16}},
          lineColor = {255, 128, 0},
          pattern = LinePattern.Dot)}),
    Documentation(info = "<html>
<p>
This partial model defines the basic interfaces required for any driveline
subsystem. This class should be extended to form a&nbsp;driveline interface
definition with the correct number of wheelHub connectors for the type of
vehicle being modelled. See the
<a href=\"modelica://VehicleInterfaces.Drivelines\">documentation</a> and
<a href=\"modelica://VehicleInterfaces.Drivelines.Tutorial\">tutorial</a>
for more information.
</p>
</html>"));
end Base;
