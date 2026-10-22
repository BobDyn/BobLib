within BobLibTest.Brakes.Internal;
model BrakeReactionSmoke "Signed reaction balance and differential brake gearing"

  inner Modelica.Mechanics.MultiBody.World world(enableAnimation = false) annotation(Placement(transformation(origin = {-350, -230}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed sprung annotation(Placement(transformation(origin = {-140, 130}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Joints.Revolute upright(
    n = {0, 1, 0},
    useAxisFlange = true) annotation(Placement(transformation(origin = {-200, 60}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed articulation(exact = true) annotation(Placement(transformation(origin = {-260, 60}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant speed(k = 1) annotation(Placement(transformation(origin = {-320, 60}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  BobLib.Utilities.Mechanics.MultiBody.InboardShaft shaft annotation(Placement(transformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque inputTorque(tau_constant = -120) annotation(Placement(transformation(origin = {-100, 150}, extent = {{-14, -14}, {14, 14}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Components.Fixed wheel annotation(Placement(transformation(origin = {-20, 60}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  BobLib.Brakes.Internal.RigAxleBrake inputBrake(
    useWheelFrames = false,
    placement = BobLib.Brakes.Types.Placement.DifferentialInput,
    inputRatio = 3, leftContactTorque = 100, rightContactTorque = 140) annotation(Placement(transformation(origin = {180, 130}, extent = {{-20, -10}, {20, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Fixed leftWheel annotation(Placement(transformation(origin = {100, 60}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Fixed rightWheel annotation(Placement(transformation(origin = {260, 60}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  BobLib.Drivelines.Internal.Differential1D actualDifferential(differentialArchitecture = BobLib.Drivelines.Types.DifferentialArchitecture.Open) annotation(Placement(transformation(origin = {-40, -150}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Components.IdealGear actualReduction(ratio = 3) annotation(Placement(transformation(origin = {-160, -150}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.ConstantTorque actualInputBrake(tau_constant = -80) annotation(Placement(transformation(origin = {-260, -150}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Speed actualLeft(exact = true) annotation(Placement(transformation(origin = {100, -100}, extent = {{-14, -14}, {14, 14}}, rotation = 180)));

  Modelica.Mechanics.Rotational.Sources.Speed actualRight(exact = true) annotation(Placement(transformation(origin = {100, -200}, extent = {{-14, -14}, {14, 14}}, rotation = 180)));

  Modelica.Blocks.Sources.Constant leftSpeed(k = 10) annotation(Placement(transformation(origin = {240, -100}, extent = {{-15, -15}, {15, 15}}, rotation = 180)));

  Modelica.Blocks.Sources.Constant rightSpeed(k = 20) annotation(Placement(transformation(origin = {240, -200}, extent = {{-15, -15}, {15, 15}}, rotation = 180)));

equation
  connect(actualInputBrake.flange, actualReduction.flange_a) annotation(Line(points = {{-246, -150}, {-174, -150}}, color = {0, 0, 0}));
  connect(actualReduction.flange_b, actualDifferential.shaft_in) annotation(Line(points = {{-146, -150}, {-60, -150}}, color = {0, 0, 0}));
  connect(actualDifferential.shaft_left, actualLeft.flange) annotation(Line(points = {{-20, -142}, {-17, -142}, {-17, -100}, {86, -100}}, color = {0, 0, 0}));
  connect(actualDifferential.shaft_right, actualRight.flange) annotation(Line(points = {{-20, -158}, {-17, -158}, {-17, -200}, {86, -200}}, color = {0, 0, 0}));
  connect(leftSpeed.y, actualLeft.w_ref) annotation(Line(points = {{223.5, -100}, {116.8, -100}}, color = {0, 0, 127}));
  connect(rightSpeed.y, actualRight.w_ref) annotation(Line(points = {{223.5, -200}, {116.8, -200}}, color = {0, 0, 127}));
  assert(abs(actualInputBrake.w-45)<1e-8, "Differential brake shaft speed has the wrong ratio");
  assert(
    abs(actualDifferential.shaft_left.tau-120)<1e-8 and abs(actualDifferential.shaft_right.tau-120)<1e-8,
    "Mechanical open differential brake torque split disagrees with the static rig");
  connect(sprung.frame_b, shaft.sprungFrame) annotation(Line(points = {{-126, 130}, {-123, 130}, {-123, 52}, {-120, 52}}, color = {95, 95, 95}));
  connect(upright.frame_a, world.frame_b) annotation(Line(points = {{-214, 60}, {-230, 60}, {-230, 20}, {-330, 20}, {-330, -230}, {-340, -230}}, color = {95, 95, 95}));
  connect(upright.frame_b, shaft.uprightFrame) annotation(Line(points = {{-186, 60}, {-183, 60}, {-183, 36}, {-77, 36}, {-77, 52}, {-80, 52}}, color = {95, 95, 95}));
  connect(articulation.flange, upright.axis) annotation(Line(points = {{-246, 60}, {-243, 60}, {-243, 80}, {-200, 80}, {-200, 74}}, color = {0, 0, 0}));
  connect(speed.y, articulation.w_ref) annotation(Line(points = {{-303.5, 60}, {-276.8, 60}}, color = {0, 0, 127}));
  connect(inputTorque.flange, shaft.inboard) annotation(Line(points = {{-100, 136}, {-115, 136}, {-115, 72}, {-120, 72}}, color = {0, 0, 0}));
  connect(shaft.outboard, wheel.flange) annotation(Line(points = {{-80, 72}, {-77, 72}, {-77, 60}, {-20, 60}}, color = {0, 0, 0}));
  connect(leftWheel.flange, inputBrake.left) annotation(Line(points = {{100, 60}, {83, 60}, {83, 130}, {160, 130}}, color = {0, 0, 0}));
  connect(rightWheel.flange, inputBrake.right) annotation(Line(points = {{260, 60}, {243, 60}, {243, 130}, {200, 130}}, color = {0, 0, 0}));
  assert(abs(shaft.transmittedTorque+120)<1e-8, "Incorrect shaft torque sign");
  assert(abs(shaft.inboard.tau+shaft.outboard.tau)<1e-8, "Shaft torque is not conserved");
  assert(
    Modelica.Math.Vectors.norm(shaft.sprungFrame.t+shaft.uprightFrame.t)<1e-8,
    "Reaction transfer creates a net external moment");
  assert(abs(shaft.uprightFrame.t[2]-120)<1e-8, "Inboard brake does not unload the upright moment");
  assert(abs(inputBrake.inputBrakeTorque+80)<1e-8, "Input brake ratio is incorrect");
  assert(
    abs(inputBrake.left.tau-120)<1e-8 and abs(inputBrake.right.tau-120)<1e-8,
    "Open differential must deliver equal wheel brake torques");
  assert(abs(inputTorque.w-1)<1e-8, "Carrier motion is missing from inboard speed");
  assert(
    abs(shaft.inboard.tau*inputTorque.w+shaft.uprightFrame.t*Modelica.Mechanics.MultiBody.Frames.angularVelocity2(shaft.uprightFrame.R)+shaft.sprungFrame.t*Modelica.Mechanics.MultiBody.Frames.angularVelocity2(shaft.sprungFrame.R))<1e-8,
    "Shaft reaction transfer does not conserve power");
  annotation(experiment(StopTime = 0.01, Tolerance = 1e-8));
  annotation(Diagram(coordinateSystem(extent = {{-360, -240}, {300, 180}}, preserveAspectRatio = true)));
end BrakeReactionSmoke;
