within BobLib.Utilities.Mechanics.Multibody;

model ChassisActuator
  import Modelica.SIunits;
  
  parameter SIunits.Position axleRef[3] "Vector from origin to axle reference, resolved in world frame";
  
  // Chassis interface
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a chassisFrame annotation(
    Placement(transformation(origin = {70, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 80}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  
  // Ground reference
  Modelica.Mechanics.MultiBody.Parts.Fixed groundFixed (animation = false, r = {0, 0, 0}) annotation(Placement(transformation(origin = {-70, -70},extent={{-10,-10},{10,10}})));
  
  // Heave actuation
  Modelica.Mechanics.MultiBody.Joints.Prismatic heaveDOF (n = {0, 0, 1}, useAxisFlange = true, animation = true) annotation(Placement(transformation(origin={50,50},extent={{10,-10},{-10,10}},rotation=180)));
  Modelica.Mechanics.Translational.Sources.Position heavePosition (exact = true, useSupport = true) annotation(Placement(transformation(origin={58,10},extent={{10,-10},{-10,10}},rotation=-90)));
  Modelica.Blocks.Interfaces.RealInput heaveInput annotation(
    Placement(transformation(origin = {116, -20}, extent = {{16, -16}, {-16, 16}}), iconTransformation(origin = {-36, 60}, extent = {{-16, -16}, {16, 16}})));
  
  // Roll actuation
  Modelica.Mechanics.MultiBody.Joints.Revolute rollDOF (n = {1, 0, 0}, useAxisFlange = true) annotation(Placement(transformation(origin={0,-30},extent={{-10,-10},{10,10}},rotation=90)));
  Modelica.Mechanics.Rotational.Sources.Position rollPosition (exact = true, useSupport = true) annotation(Placement(transformation(origin = {-50, -30},extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Interfaces.RealInput rollInput annotation(
    Placement(transformation(origin = {-116, -30}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-36, -60}, extent = {{-16, -16}, {16, 16}})));
  
  // Instrumentation
  Modelica.Mechanics.MultiBody.Sensors.CutForce sprungLoads (animation = false) annotation(Placement(transformation(origin={-30,-70},extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Interfaces.RealOutput jackingOutput annotation(
    Placement(transformation(origin = {-38, -116}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {36, 0}, extent = {{-16, -16}, {16, 16}})));

protected
  Modelica.Mechanics.MultiBody.Parts.FixedRotation toAxle (r = axleRef, animation = false) annotation(Placement(transformation(origin={0,30},extent={{-10,-10},{10,10}},rotation=90)));
public
equation
  connect(groundFixed.frame_b, sprungLoads.frame_a) annotation(
    Line(points = {{-60, -70}, {-40, -70}}, color = {95, 95, 95}));
  connect(sprungLoads.frame_b, rollDOF.frame_a) annotation(
    Line(points = {{-20, -70}, {0, -70}, {0, -40}}, color = {95, 95, 95}));
  connect(rollPosition.support, rollDOF.support) annotation(
    Line(points = {{-50, -40}, {-10, -40}, {-10, -36}}));
  connect(rollDOF.axis, rollPosition.flange) annotation(
    Line(points = {{-10, -30}, {-40, -30}}));
  connect(toAxle.frame_a, rollDOF.frame_b) annotation(
    Line(points = {{0, 20}, {0, -20}}, color = {95, 95, 95}));
  connect(heaveDOF.frame_a, toAxle.frame_b) annotation(
    Line(points = {{40, 50}, {0, 50}, {0, 40}}, color = {95, 95, 95}));
  connect(heavePosition.support, heaveDOF.support) annotation(
    Line(points = {{48, 10}, {46, 10}, {46, 44}}, color = {0, 127, 0}));
  connect(heaveDOF.axis, heavePosition.flange) annotation(
    Line(points = {{58, 44}, {58, 20}}, color = {0, 127, 0}));
  connect(heaveDOF.frame_b, chassisFrame) annotation(
    Line(points = {{60, 50}, {70, 50}, {70, 100}}, color = {95, 95, 95}));
  connect(rollInput, rollPosition.phi_ref) annotation(
    Line(points = {{-116, -30}, {-62, -30}}, color = {0, 0, 127}));
  connect(heaveInput, heavePosition.s_ref) annotation(
    Line(points = {{116, -20}, {58, -20}, {58, -2}}, color = {0, 0, 127}));
  connect(sprungLoads.force[3], jackingOutput) annotation(
    Line(points = {{-38, -80}, {-38, -116}}, color = {0, 0, 127}));
annotation(
    Icon(graphics = {Text(origin = {-80, 92}, extent = {{-40, 20}, {40, -20}}, textString = "Heave"), Text(origin = {-70, -28}, extent = {{-30, 20}, {30, -20}}, textString = "Roll"), Text(origin = {82, -20}, extent = {{-40, 20}, {40, -20}}, textString = "Jacking"), Rectangle( fillColor = {0, 170, 0}, fillPattern = FillPattern.Solid, extent = {{-20, 80}, {20, -80}})}),
    Diagram(graphics));
end ChassisActuator;