within BobLib.Brakes.Internal;
model RigAxleBrake "Static brake loading for a prescribed four-post sweep"

  parameter BobLib.Brakes.Types.Placement placement = BobLib.Brakes.Types.Placement.Outboard "Brake location determining the upright and sprung-frame reaction paths" annotation(Dialog(group = "Architecture"));
  parameter Real inputRatio(min = Modelica.Constants.small) = 1 "Brake shaft speed / differential carrier speed" annotation(Dialog(group = "Operating settings"));
  input Modelica.Units.SI.Torque leftContactTorque;
  input Modelica.Units.SI.Torque rightContactTorque;
  Modelica.Mechanics.Rotational.Interfaces.Flange_b left annotation(Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b right annotation(Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));
  parameter Boolean useWheelFrames = true "Expose explicit upright or sprung caliper supports in a MultiBody rig" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  final parameter Boolean transferReaction = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard "Transfer the brake reaction to the sprung frame for an inboard rig brake";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungFrame if transferReaction "Sprung support for halfshaft or differential braking" annotation(Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a leftWheelFrame if useWheelFrames "Left unsprung upright; caliper support for outboard braking" annotation(Placement(transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a rightWheelFrame if useWheelFrames "Right unsprung upright; caliper support for outboard braking" annotation(Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.InboardShaft leftShaft(
    useSpatialSupports = true,
    accountForCarrierMotion = false) if transferReaction annotation(Placement(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, transformation(origin = {-70, -25}, extent = {{10, -10}, {-10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.InboardShaft rightShaft(
    useSpatialSupports = true,
    accountForCarrierMotion = false) if transferReaction annotation(Placement(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, transformation(origin = {70, -25}, extent = {{-10, -10}, {10, 10}})));
  output Modelica.Units.SI.Torque inputBrakeTorque = if placement==BobLib.Brakes.Types.Placement.DifferentialInput then -(leftContactTorque+rightContactTorque)/inputRatio
    else 0
    "Signed torque at the single brake shaft, before the reduction";

  Modelica.Mechanics.Rotational.Sources.Torque leftBrake(useSupport = useWheelFrames) annotation(Placement(transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Torque rightBrake(useSupport = useWheelFrames) annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D leftCaliper(n = {0, 1, 0}) if useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput annotation(
    Placement(
      visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
      transformation(origin = {-45, -65}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D rightCaliper(n = {0, 1, 0}) if useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput annotation(
    Placement(
      visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
      transformation(origin = {45, -65}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D differentialHousing(n = {0, 1, 0})
    if useWheelFrames and placement==BobLib.Brakes.Types.Placement.DifferentialInput
    "Combined brake and reduction housing reaction in the ideal static rig" annotation(
    Placement(visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.DifferentialInput,
      transformation(origin = {0, -65}, extent = {{-10, -10}, {10, 10}})));

protected
  outer Modelica.Mechanics.MultiBody.World world;

equation
  assert(not useWheelFrames or world.driveTrainMechanics3D,
    "RigAxleBrake: explicit caliper supports require world.driveTrainMechanics3D=true");
  connect(
    leftBrake.support,
    leftCaliper.flange_b) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
    points = {{-50, -10}, {-50, -18}, {-25, -18}, {-25, -65}, {-35, -65}}, color = {0, 0, 0}));
  connect(
    rightBrake.support,
    rightCaliper.flange_b) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
    points = {{50, -10}, {50, -18}, {25, -18}, {25, -65}, {35, -65}}, color = {0, 0, 0}));

  connect(leftBrake.support, differentialHousing.flange_b) annotation(Line(
    visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.DifferentialInput,
    points = {{-50, -10}, {-50, -18}, {-25, -18}, {-25, -45}, {25, -45}, {25, -65}, {10, -65}}, color = {0, 0, 0}));
  connect(rightBrake.support, differentialHousing.flange_b) annotation(Line(
    visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.DifferentialInput,
    points = {{50, -10}, {50, -18}, {25, -18}, {25, -65}, {10, -65}}, color = {0, 0, 0}));
  connect(differentialHousing.frame_a, sprungFrame) annotation(Line(
    visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.DifferentialInput,
    points = {{0, -75}, {0, -100}}, color = {95, 95, 95}));

  if placement == BobLib.Brakes.Types.Placement.Outboard then
    connect(
      leftCaliper.frame_a,
      leftWheelFrame) annotation(Line(visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.Outboard,
      points = {{-45, -75}, {-45, -85}, {-90, -85}, {-90, -40}, {-100, -40}}, color = {95, 95, 95}));
    connect(
      rightCaliper.frame_a,
      rightWheelFrame) annotation(Line(visible = useWheelFrames and placement==BobLib.Brakes.Types.Placement.Outboard,
      points = {{45, -75}, {45, -85}, {90, -85}, {90, -40}, {100, -40}}, color = {95, 95, 95}));
  else
    connect(
      leftCaliper.frame_a,
      sprungFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
      points = {{-45, -75}, {-45, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
    connect(
      rightCaliper.frame_a,
      sprungFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard and placement<>BobLib.Brakes.Types.Placement.DifferentialInput,
      points = {{45, -75}, {45, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  end if;
  leftBrake.tau = if placement==BobLib.Brakes.Types.Placement.DifferentialInput then inputRatio*inputBrakeTorque/2
    else -leftContactTorque;
  rightBrake.tau = if placement==BobLib.Brakes.Types.Placement.DifferentialInput then inputRatio*inputBrakeTorque/2
    else -rightContactTorque;

  if transferReaction then
    connect(
      leftBrake.flange,
      leftShaft.inboard) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{-60, 0}, {-60, -19}}));
    connect(
      rightBrake.flange,
      rightShaft.inboard) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{60, 0}, {60, -19}}));
    connect(
      leftShaft.outboard,
      left) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{-80, -19}, {-90, -19}, {-90, 0}, {-100, 0}}));
    connect(
      rightShaft.outboard,
      right) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{80, -19}, {90, -19}, {90, 0}, {100, 0}}));
  else
    connect(
      leftBrake.flange,
      left) annotation(Line(visible = not useWheelFrames or placement==BobLib.Brakes.Types.Placement.Outboard, points = {{-60, 0}, {-100, 0}}));
    connect(
      rightBrake.flange,
      right) annotation(Line(visible = not useWheelFrames or placement==BobLib.Brakes.Types.Placement.Outboard, points = {{60, 0}, {100, 0}}));
  end if;
  connect(
    leftShaft.sprungFrame,
    sprungFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{-60, -29}, {-52, -29}, {-52, -45}, {-65, -45}, {-65, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    rightShaft.sprungFrame,
    sprungFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{60, -29}, {52, -29}, {52, -45}, {65, -45}, {65, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    leftShaft.uprightFrame,
    leftWheelFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{-80, -29}, {-90, -29}, {-90, -40}, {-100, -40}}, color = {95, 95, 95}));
  connect(
    rightShaft.uprightFrame,
    rightWheelFrame) annotation(Line(visible = useWheelFrames and placement<>BobLib.Brakes.Types.Placement.Outboard, points = {{80, -29}, {90, -29}, {90, -40}, {100, -40}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -50}, {100, 50}}), graphics = {
    Line(points = {{-100, 0}, {100, 0}}, thickness = 2),
    Ellipse(visible = placement<>BobLib.Brakes.Types.Placement.DifferentialInput, extent = {{-70, -35}, {-10, 35}}, fillPattern = FillPattern.Solid, fillColor = {175, 175, 175}),
    Ellipse(visible = placement<>BobLib.Brakes.Types.Placement.DifferentialInput, extent = {{10, -35}, {70, 35}}, fillPattern = FillPattern.Solid, fillColor = {175, 175, 175}),
    Ellipse(visible = placement==BobLib.Brakes.Types.Placement.DifferentialInput, extent = {{-35, -35}, {35, 35}}, fillPattern = FillPattern.Solid, fillColor = {175, 175, 175}),
    Text(extent = {{-100, 80}, {100, 55}}, textString = "%name", textColor = {0, 0, 255})}),
    Diagram(coordinateSystem(extent = {{-100, -100}, {100, 20}})),
    Documentation(info = "<html><p>With useWheelFrames, leftWheelFrame and rightWheelFrame expose the non-spinning uprights. Outboard calipers react directly through these frames. Inboard and differential-input calipers react through the conditional sprungFrame; shaft adapters transport torque and redirect its axis without adding a second upright reaction. The containing rig must enable world.driveTrainMechanics3D. Disabled support wires are hidden.</p><p>Balances the signed contact-patch spin torques for prescribed, non-spinning rig wheels. Per-wheel brakes balance each wheel independently. The static equations of an ideal reduction and open differential distribute a single input brake torque equally between the two outputs; unequal contact torques therefore leave a residual at the uprights. Input-shaft torque is reported as inputBrakeTorque. The single differentialHousing support includes both input-brake and ideal-reduction reactions; its total moment is therefore the sum of the output-shaft torques, not merely inputBrakeTorque. The static rig does not need extra differential shaft states; the vehicle uses its actual driveline differential. This is a static load case, not a friction or ABS controller.</p></html>"));
end RigAxleBrake;
