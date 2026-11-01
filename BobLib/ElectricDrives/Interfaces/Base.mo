within BobLib.ElectricDrives.Interfaces;
partial model Base "Basic interface for an electric drives"

  // Adapted from VehicleInterfaces 2.0.2: typed BobLib control bus.
  // Copyright (C) 2005-2013 Dassault Systemes, DLR and Modelon;
  // (C) 2013-2025 Modelica Association. Modelica License 1.1:
  // Resources/Licenses/LICENSE_VehicleInterfaces.txt.

  parameter Boolean usingMultiBodySystem = false
    "= true, if connecting to a MultiBody system"
    annotation(choices(checkBox = true), Dialog(tab = "Advanced", group = "Only for 1D electric drive", enable = not includeBearing));

protected
  parameter Boolean includeBearing = false "Include bearing connector";

public
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chassisFrame
    if includeBearing or usingMultiBodySystem
    "Chassis attachment for the motor's internal mount"
    annotation(Placement(transformation(origin = {0, 100}, extent = {{-8, -8}, {8, 8}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Interfaces.FlangeWithBearing shaft_b(
    includeBearingConnector = includeBearing or usingMultiBodySystem)
    "Mechanical shaft" annotation(Placement(transformation(extent = {{90, -10},
            {110, 10}})));
  BobLib.Utilities.Interfaces.ControlBus controlBus "Control signal bus"
    annotation(Placement(transformation(
        origin = {-100, -60},
        extent = {{-20, -20}, {20, 20}},
        rotation = 90), iconTransformation(
        origin = {100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

  VehicleInterfaces.Mechanics.MultiBody.MultiBodyEnd end_1(
    includeBearingConnector = includeBearing or usingMultiBodySystem)
    annotation(Placement(transformation(
        origin = {100, 22},
        extent = {{-8, -6}, {8, 6}},
        rotation = 270)));

  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p
    "Positive electrical pin" annotation(Placement(transformation(extent = {
            {50, 90}, {70, 110}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n
    "Negative electrical pin" annotation(Placement(transformation(extent = {
            {-70, 90}, {-50, 110}})));

equation
  connect(end_1.flange, shaft_b) annotation(Line(
      points = {{100, 19.3333}, {100, 0}},
      color = {135, 135, 135},
      thickness = 0.5));

  annotation(
    Documentation(info = "<html>
<p>
This partial model defines the interfaces required for an electric machine
subsystem. This class should be extended to form an electric machine
interface definition with propper electrical connectors. See the
<a href=\"modelica://VehicleInterfaces.ElectricDrives\">documentation</a>
and
<a href=\"modelica://VehicleInterfaces.ElectricDrives.Tutorial\">tutorial</a>
for more information.
</p>
</html>"));
end Base;
