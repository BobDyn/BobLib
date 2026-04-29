within BobLib.Standards.Templates;

partial model KinFMI
  import Modelica.SIunits;
  import BobLib.Resources.StandardRecord.KnCRecord;

  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;

  inner Modelica.Mechanics.MultiBody.World world(g = 0, n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));
  
  input Real heaveInput;
  input Real rollInput;
  
  Modelica.Blocks.Sources.RealExpression rollSource(y = rollInput)  annotation(
    Placement(transformation(origin = {-40, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression heaveSource(y = heaveInput)  annotation(
    Placement(transformation(origin = {40, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

protected
  // Ground fixture
  Modelica.Mechanics.MultiBody.Parts.Fixed groundFixed(r = {0, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Roll DOF
  Modelica.Mechanics.MultiBody.Joints.Revolute rollDOF(n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position rollPosition(useSupport = true, exact = true) annotation(
    Placement(transformation(origin = {-22, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));

  // Heave DOF
  Modelica.Mechanics.MultiBody.Joints.Prismatic heaveDOF(useAxisFlange = true, n = {0, 0, 1}, e) annotation(
    Placement(transformation(origin = {0, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.Translational.Sources.Position heavePosition(useSupport = true, exact = true) annotation(
    Placement(transformation(origin = {20, 16}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  // Kinematics to axle
  Modelica.Mechanics.MultiBody.Parts.FixedRotation toAxle(animation = false) annotation(
    Placement(transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Contact patch references
  Modelica.Mechanics.MultiBody.Parts.Fixed leftCPFixed(animation = false) annotation(
    Placement(transformation(origin = {-120, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed rightCPFixed(animation = false) annotation(
    Placement(transformation(origin = {120, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  // Instrumentation
  Modelica.Mechanics.MultiBody.Sensors.CutForce sprungLoads(animation = false) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Left jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-120, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical left_DOF_xyz(sphereDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute leftRevolute(n = {1, 0, 0}, useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Disc leftAngleOffset(deltaPhi = Modelica.Constants.pi/2) annotation(
    Placement(transformation(origin = {-71, -5}, extent = {{-5, -5}, {5, 5}})));

  // Right jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {120, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {100, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical right_DOF_xyz(sphereDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {40, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute rightRevolute(useAxisFlange = true, n = {1, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {70, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.Rotational.Components.Disc rightAngleOffset(deltaPhi = -Modelica.Constants.pi/2) annotation(
    Placement(transformation(origin = {71, -5}, extent = {{5, -5}, {-5, 5}})));

equation
  connect(groundFixed.frame_b, sprungLoads.frame_a) annotation(
    Line(points = {{0, -100}, {0, -80}}, color = {95, 95, 95}));
  connect(rollPosition.support, rollDOF.support) annotation(
    Line(points = {{-12, -50}, {-10, -50}, {-10, -46}}));
  connect(rollPosition.flange, rollDOF.axis) annotation(
    Line(points = {{-22, -40}, {-10, -40}}));
  connect(sprungLoads.frame_b, rollDOF.frame_a) annotation(
    Line(points = {{0, -60}, {0, -50}}, color = {95, 95, 95}));
  connect(leftCPFixed.frame_b, left_DOF_x.frame_a) annotation(
    Line(points = {{-120, -70}, {-120, -60}}, color = {95, 95, 95}));
  connect(left_DOF_x.frame_b, left_DOF_y.frame_a) annotation(
    Line(points = {{-120, -40}, {-120, -30}, {-110, -30}}, color = {95, 95, 95}));
  connect(rightCPFixed.frame_b, right_DOF_x.frame_a) annotation(
    Line(points = {{120, -70}, {120, -60}}, color = {95, 95, 95}));
  connect(right_DOF_x.frame_b, right_DOF_y.frame_a) annotation(
    Line(points = {{120, -40}, {120, -30}, {110, -30}}, color = {95, 95, 95}));
  connect(leftRevolute.support, leftAngleOffset.flange_a) annotation(
    Line(points = {{-76, -20}, {-76, -4}}));
  connect(rightAngleOffset.flange_a, rightRevolute.support) annotation(
    Line(points = {{76, -5}, {76, -21}}));
  connect(rightAngleOffset.flange_b, rightRevolute.axis) annotation(
    Line(points = {{66, -5}, {60, -5}, {60, -21}, {70, -21}}));
  connect(leftAngleOffset.flange_b, leftRevolute.axis) annotation(
    Line(points = {{-66, -5}, {-60, -5}, {-60, -21}, {-70, -21}}));
  connect(left_DOF_y.frame_b, leftRevolute.frame_a) annotation(
    Line(points = {{-90, -30}, {-80, -30}}, color = {95, 95, 95}));
  connect(right_DOF_y.frame_b, rightRevolute.frame_a) annotation(
    Line(points = {{90, -30}, {80, -30}}, color = {95, 95, 95}));
  connect(leftRevolute.frame_b, left_DOF_xyz.frame_a) annotation(
    Line(points = {{-60, -30}, {-40, -30}, {-40, -20}}, color = {95, 95, 95}));
  connect(rightRevolute.frame_b, right_DOF_xyz.frame_a) annotation(
    Line(points = {{60, -30}, {40, -30}, {40, -20}}, color = {95, 95, 95}));
  connect(heavePosition.support, heaveDOF.support) annotation(
    Line(points = {{10, 16}, {6, 16}}, color = {0, 127, 0}));
  connect(heavePosition.flange, heaveDOF.axis) annotation(
    Line(points = {{20, 26}, {20, 28}, {6, 28}}, color = {0, 127, 0}));
  connect(rollDOF.frame_b, toAxle.frame_a) annotation(
    Line(points = {{0, -30}, {0, -20}}, color = {95, 95, 95}));
  connect(toAxle.frame_b, heaveDOF.frame_a) annotation(
    Line(points = {{0, 0}, {0, 10}}, color = {95, 95, 95}));
  connect(rollSource.y, rollPosition.phi_ref) annotation(
    Line(points = {{-28, -70}, {-22, -70}, {-22, -62}}, color = {0, 0, 127}));
  connect(heaveSource.y, heavePosition.s_ref) annotation(
    Line(points = {{30, -50}, {20, -50}, {20, 4}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    Icon(coordinateSystem(extent = {{-140, -120}, {140, 120}})));
end KinFMI;
