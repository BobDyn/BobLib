within BobLib.Standards.Templates;

partial model KnC
  import Modelica.SIunits;
  
  // External inputs (FMU / external driver)
  input SIunits.Angle pinionAngle_in;
  input SIunits.Angle rollAngle_in;
  input SIunits.Force leftFx_in;
  input SIunits.Force leftFy_in;
  input SIunits.Force rightFx_in;
  input SIunits.Force rightFy_in;
  
  inner Modelica.Mechanics.MultiBody.World world(g = 0, n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  output Real leftGamma;
  output Real leftToe;
  output Real leftCaster;
  output Real leftKpi;
  output Real leftMechTrail;
  output Real leftMechScrub;

  output Real rightGamma;
  output Real rightToe;
  output Real rightCaster;
  output Real rightKpi;
  output Real rightMechTrail;
  output Real rightMechScrub;

  output Real jackingForce;
  
  
protected
  Real leftDeltaVec[3];
  Real leftKingpinVec[3];
  Real leftGroundParam;
  Real leftGroundPoint[3];
  Real rightDeltaVec[3];
  Real rightKingpinVec[3];
  Real rightGroundParam;
  Real rightGroundPoint[3];
  
  // Routed signals used by the model / inheriting models
  SIunits.Angle pinionAngle;
  SIunits.Angle rollAngle;
  SIunits.Force leftFx;
  SIunits.Force leftFy;
  SIunits.Force rightFx;
  SIunits.Force rightFy;
  
  parameter Boolean useInternalInput = true;
  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter SIunits.Length rackPosition = 1*0.0254 "Rack position";
  parameter SIunits.Force forceAmplitude = 100 "Force amplitude";
  
  // Time-value tables [time, normalized value]
  final parameter Real leftFxTable[:, 2] = [
    0.0, 0;
    0.1, 0;
    0.2, 1;
    0.3, 0;
    0.4, 0;
    0.5, 1
  ];
  
  final parameter Real rightFxTable[:, 2] = leftFxTable;
  
  final parameter Real leftFyTable[:, 2] = [
    0.0, 0;
    0.1, 0;
    0.2, 0;
    0.3, 0;
    0.4, 1;
    0.5, 0
  ];
  
  final parameter Real rightFyTable[:, 2] = leftFyTable;
  
  // Ground fixture
  Modelica.Mechanics.MultiBody.Parts.Fixed groundFixed(r = {0, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Origin roll frame
  Modelica.Mechanics.Rotational.Sources.Position rollPosition(useSupport = true, exact = true) annotation(
    Placement(transformation(origin = {-22, -50}, extent = {{-10, -10}, {10, 10}}, rotation = -270)));
  Modelica.Mechanics.MultiBody.Joints.Revolute rollDOF(n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Contact patch references
  Modelica.Mechanics.MultiBody.Parts.Fixed leftCPFixed(animation = false)  annotation(
    Placement(transformation(origin = {-140, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed rightCPFixed(animation = false)  annotation(
    Placement(transformation(origin = {140, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // Left contact patch forces
  Modelica.Blocks.Sources.CombiTimeTable leftFxSource(
    columns = {2},
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table = leftFxTable) annotation(
    Placement(transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.CombiTimeTable leftFySource(
    columns = {2},
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table = leftFyTable) annotation(
    Placement(transformation(origin = {-130, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant leftFzSource(k = 0) annotation(
    Placement(transformation(origin = {-130, 20}, extent = {{-10, -10}, {10, 10}})));
  final Modelica.Mechanics.MultiBody.Forces.WorldForce leftCPForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
    Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
  
  // Right contact patch forces
  Modelica.Blocks.Sources.CombiTimeTable rightFxSource(
    columns = {2},
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table = rightFxTable) annotation(
    Placement(transformation(origin = {130, 80}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.CombiTimeTable rightFySource(
    columns = {2},
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table = rightFyTable) annotation(
    Placement(transformation(origin = {130, 50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Constant rightFzSource(k = 0) annotation(
    Placement(transformation(origin = {130, 20}, extent = {{10, -10}, {-10, 10}})));
  final Modelica.Mechanics.MultiBody.Forces.WorldForce rightCPForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
    Placement(transformation(origin = {70, 20}, extent = {{10, -10}, {-10, 10}})));
  
  // Instrumentation
  Modelica.Mechanics.MultiBody.Sensors.CutForce sprungLoads(animation = false) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Left jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-140, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic left_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical left_DOF_xyz(sphereDiameter = jointDiameter)   annotation(
    Placement(transformation(origin = {-40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute leftRevolute(n = {1, 0, 0}, useAxisFlange = true, animation = false)  annotation(
    Placement(transformation(origin = {-80, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Disc leftAngleOffset(deltaPhi = Modelica.Constants.pi / 2)  annotation(
    Placement(transformation(origin = {-81, -5}, extent = {{-5, -5}, {5, 5}})));
  
  // Right jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {140, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic right_DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {110, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical right_DOF_xyz(sphereDiameter = jointDiameter)  annotation(
    Placement(transformation(origin = {40, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute rightRevolute(useAxisFlange = true, n = {1, 0, 0}, animation = false)  annotation(
    Placement(transformation(origin = {80, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Rotational.Components.Disc rightAngleOffset(deltaPhi = -Modelica.Constants.pi / 2)  annotation(
    Placement(transformation(origin = {81, -5}, extent = {{5, -5}, {-5, 5}}, rotation = -0)));

  // Internal roll input
  Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 2*Modelica.Constants.pi/180, startTime = 1) annotation(
    Placement(transformation(origin = {-50, -70}, extent = {{-10, -10}, {10, 10}})));
  
  // Internal routed signals
  SIunits.Angle rollAngle_internal;
  SIunits.Force leftFx_internal;
  SIunits.Force leftFy_internal;
  SIunits.Force rightFx_internal;
  SIunits.Force rightFy_internal;
  
equation
  // Instrumentation
  
  jackingForce = sprungLoads.force[3];
  
  // Internal signal definitions
  rollAngle_internal = ramp.y;
  leftFx_internal = forceAmplitude*leftFxSource.y[1];
  leftFy_internal = forceAmplitude*leftFySource.y[1];
  rightFx_internal = forceAmplitude*rightFxSource.y[1];
  rightFy_internal = forceAmplitude*rightFySource.y[1];
  
  // Routed external/internal signals
  pinionAngle = pinionAngle_in;
  rollAngle = if useInternalInput then rollAngle_internal else rollAngle_in;
  leftFx = if useInternalInput then leftFx_internal else leftFx_in;
  leftFy = if useInternalInput then leftFy_internal else leftFy_in;
  rightFx = if useInternalInput then rightFx_internal else rightFx_in;
  rightFy = if useInternalInput then rightFy_internal else rightFy_in;
  
  // Drive model inputs
  rollPosition.phi_ref = rollAngle;
  leftCPForce.force[1] = leftFx;
  leftCPForce.force[2] = leftFy;
  leftCPForce.force[3] = leftFzSource.y;
  rightCPForce.force[1] = rightFx;
  rightCPForce.force[2] = rightFy;
  rightCPForce.force[3] = rightFzSource.y;
  
  connect(groundFixed.frame_b, sprungLoads.frame_a) annotation(
    Line(points = {{0, -100}, {0, -80}}, color = {95, 95, 95}));
  connect(rollPosition.support, rollDOF.support) annotation(
    Line(points = {{-12, -50}, {-10, -50}, {-10, -46}}));
  connect(rollPosition.flange, rollDOF.axis) annotation(
    Line(points = {{-22, -40}, {-10, -40}}));
  connect(sprungLoads.frame_b, rollDOF.frame_a) annotation(
    Line(points = {{0, -60}, {0, -50}}, color = {95, 95, 95}));
  connect(leftCPFixed.frame_b, left_DOF_x.frame_a) annotation(
    Line(points = {{-140, -70}, {-140, -60}}, color = {95, 95, 95}));
  connect(left_DOF_x.frame_b, left_DOF_y.frame_a) annotation(
    Line(points = {{-140, -40}, {-140, -30}, {-120, -30}}, color = {95, 95, 95}));
  connect(rightCPFixed.frame_b, right_DOF_x.frame_a) annotation(
    Line(points = {{140, -70}, {140, -60}}, color = {95, 95, 95}));
  connect(right_DOF_x.frame_b, right_DOF_y.frame_a) annotation(
    Line(points = {{140, -40}, {140, -30}, {120, -30}}, color = {95, 95, 95}));
  connect(leftRevolute.support, leftAngleOffset.flange_a) annotation(
    Line(points = {{-86, -20}, {-86, -4}}));
  connect(rightAngleOffset.flange_a, rightRevolute.support) annotation(
    Line(points = {{86, -4}, {86, -20}}));
  connect(rightAngleOffset.flange_b, rightRevolute.axis) annotation(
    Line(points = {{76, -4}, {70, -4}, {70, -20}, {80, -20}}));
  connect(leftAngleOffset.flange_b, leftRevolute.axis) annotation(
    Line(points = {{-76, -4}, {-70, -4}, {-70, -20}, {-80, -20}}));
  connect(left_DOF_y.frame_b, leftRevolute.frame_a) annotation(
    Line(points = {{-100, -30}, {-90, -30}}, color = {95, 95, 95}));
  connect(right_DOF_y.frame_b, rightRevolute.frame_a) annotation(
    Line(points = {{100, -30}, {90, -30}}, color = {95, 95, 95}));
  connect(leftRevolute.frame_b, left_DOF_xyz.frame_a) annotation(
    Line(points = {{-70, -30}, {-40, -30}, {-40, -20}}, color = {95, 95, 95}));
  connect(rightRevolute.frame_b, right_DOF_xyz.frame_a) annotation(
    Line(points = {{70, -30}, {40, -30}, {40, -20}}, color = {95, 95, 95}));
  annotation();
end KnC;
