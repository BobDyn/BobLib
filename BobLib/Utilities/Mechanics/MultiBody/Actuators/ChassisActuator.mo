within BobLib.Utilities.Mechanics.MultiBody.Actuators;

model ChassisActuator

  import Modelica.Units.SI;

  parameter SI.Position axleRef[3] "Vector from origin to axle reference, resolved in world frame" annotation(Dialog(group = "Operating settings"));
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";

  // Chassis interface
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chassisFrame annotation(
    Placement(
      transformation(origin = {60, 140}, extent = {{-16, -16}, {16, 16}}, rotation = 0),
      iconTransformation(origin = {0, 80}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

  // Ground reference
  Modelica.Mechanics.MultiBody.Parts.Fixed groundFixed(
    animation = false,
    r = {0, 0, 0}) annotation(Placement(transformation(origin = {-140, -100}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  // Heave actuation
  Modelica.Mechanics.MultiBody.Joints.Prismatic heaveDOF(
    n = {0, 0, 1},
    useAxisFlange = true,
    animation = not headless) annotation(Placement(transformation(origin = {60, 90}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));

  Modelica.Mechanics.Translational.Sources.Position heavePosition(
    exact = true,
    useSupport = true) annotation(Placement(transformation(origin = {-70, 90}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealInput heaveInput(quantity = "Position", unit = "m") annotation(
    Placement(
      transformation(origin = {-180, 90}, extent = {{-16, -16}, {16, 16}}, rotation = 0),
      iconTransformation(origin = {-36, 60}, extent = {{-16, -16}, {16, 16}})));

  // Roll actuation
  Modelica.Mechanics.MultiBody.Joints.Revolute rollDOF(
    n = {1, 0, 0},
    useAxisFlange = true,
    animation = not headless) annotation(Placement(transformation(origin = {60, -50}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));

  Modelica.Mechanics.Rotational.Sources.Position rollPosition(
    exact = true,
    useSupport = true) annotation(Placement(transformation(origin = {-70, -50}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealInput rollInput(quantity = "Angle", unit = "rad") annotation(
    Placement(
      transformation(origin = {-180, -50}, extent = {{-16, -16}, {16, 16}}, rotation = 0),
      iconTransformation(origin = {-36, -60}, extent = {{-16, -16}, {16, 16}})));

  // Instrumentation
  Modelica.Mechanics.MultiBody.Sensors.CutForce sprungLoads(animation = false) annotation(Placement(transformation(origin = {-80, -100}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealOutput jackingOutput(quantity = "Force", unit = "N") annotation(
    Placement(
      transformation(origin = {180, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0),
      iconTransformation(origin = {36, 0}, extent = {{-16, -16}, {16, 16}})));

protected
  Modelica.Mechanics.MultiBody.Parts.FixedRotation toAxle(
    r = axleRef,
    animation = false) annotation(Placement(transformation(origin = {60, 10}, extent = {{-15, -15}, {15, 15}}, rotation = 90)));

public

equation
  connect(groundFixed.frame_b, sprungLoads.frame_a) annotation(
    Line(points = {{-125, -100}, {-95, -100}}, color = {95, 95, 95}));
  connect(sprungLoads.frame_b, rollDOF.frame_a) annotation(
    Line(points = {{-65, -100}, {60, -100}, {60, -65}}, color = {95, 95, 95}));
  connect(rollPosition.support, rollDOF.support) annotation(
    Line(points = {{-70, -65}, {30, -65}, {30, -59}, {45, -59}}));
  connect(rollDOF.axis, rollPosition.flange) annotation(
    Line(points = {{45, -50}, {-55, -50}}));
  connect(toAxle.frame_a, rollDOF.frame_b) annotation(
    Line(points = {{60, -5}, {60, -35}}, color = {95, 95, 95}));
  connect(heaveDOF.frame_a, toAxle.frame_b) annotation(
    Line(points = {{60, 75}, {60, 25}}, color = {95, 95, 95}));
  connect(heavePosition.support, heaveDOF.support) annotation(
    Line(points = {{-70, 75}, {30, 75}, {30, 84}, {51, 84}}, color = {0, 127, 0}));
  connect(heaveDOF.axis, heavePosition.flange) annotation(
    Line(points = {{51, 102}, {40, 102}, {40, 90}, {-55, 90}}, color = {0, 127, 0}));
  connect(heaveDOF.frame_b, chassisFrame) annotation(
    Line(points = {{60, 105}, {60, 140}}, color = {95, 95, 95}));
  connect(rollInput, rollPosition.phi_ref) annotation(
    Line(points = {{-180, -50}, {-88, -50}}, color = {0, 0, 127}));
  connect(heaveInput, heavePosition.s_ref) annotation(
    Line(points = {{-180, 90}, {-88, 90}}, color = {0, 0, 127}));
  connect(sprungLoads.force[3], jackingOutput) annotation(
    Line(points = {{-92, -115}, {160, -115}, {160, -100}, {180, -100}}, color = {0, 0, 127}));
annotation(
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}})),
    Icon(graphics = {Text(extent = {{-300, 130}, {300, 105}}, textString = "%name", textColor = {0, 0, 255}, fontSize = 0),    Rectangle(fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-20, 80}, {20, -80}})}),
    Documentation(info = "<html>
<p>
Model <code>ChassisActuator</code> provides a reusable MultiBody fixture and
actuation interface for rig-level suspension tests.
</p>
<p>
Four-post templates use it to impose heave and roll motion at an axle reference
while preserving the chassis-frame connection and measuring jacking load.
</p>
</html>"));
end ChassisActuator;