within BobLib.Utilities.Mechanics.MultiBody;
model InboardShaft "Ideal 1:1 shaft with reaction transferred from upright to sprung structure"

  parameter Boolean useSpatialSupports = false
    "Rotor1D and explicit actuator supports own reactions; only redirect the torque axis here" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean accountForCarrierMotion = true "Include carrier motion in shaft speed; disable only for static load sweeps" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Modelica.Mechanics.MultiBody.Types.Axis inboardAxis = {0, 1, 0}
    "Inboard shaft axis resolved in sprungFrame" annotation(Dialog(group = "Operating settings"), Evaluate = true);
  Modelica.Mechanics.Rotational.Interfaces.Flange_a inboard annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b outboard annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungFrame annotation(
    Placement(transformation(origin = {-100, -40}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b uprightFrame annotation(
    Placement(transformation(origin = {100, -40}, extent = {{-16, -16}, {16, 16}})));
  output Modelica.Units.SI.Torque transmittedTorque = inboard.tau;
  Modelica.Units.SI.Angle articulationAngle(start = 0, fixed = true)
    "Integrated relative carrier rotation about the transported wheel axis";
  Modelica.Units.SI.AngularVelocity articulationSpeed;
  Real spinAxisWorld[3];

  Modelica.Blocks.Sources.RealExpression reactionCommand[3](y = {0, inboard.tau, 0}) if not useSpatialSupports annotation(
    Placement(visible = not useSpatialSupports, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.Torque reaction(
    animation = false,
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.frame_b) if not useSpatialSupports annotation(
    Placement(visible = not useSpatialSupports, transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression axisReactionCommand[3](y = inboard.tau*(
    Modelica.Math.Vectors.normalizeWithAssert(inboardAxis)-
    Modelica.Mechanics.MultiBody.Frames.resolve2(sprungFrame.R, spinAxisWorld)))
    if useSpatialSupports annotation(
    Placement(
      visible = useSpatialSupports,
      transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Mechanics.MultiBody.Forces.WorldTorque axisReaction(
    animation = false, resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)
    if useSpatialSupports annotation(
    Placement(visible = useSpatialSupports, transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}})));

equation

  if useSpatialSupports then
    uprightFrame.f = zeros(3);
    uprightFrame.t = zeros(3);
  end if;
  connect(axisReaction.frame_b, sprungFrame) annotation(
    Line(visible = useSpatialSupports, points = {{-10, -40}, {-100, -40}}, color = {95, 95, 95}));
  connect(axisReactionCommand.y, axisReaction.torque) annotation(
    Line(visible = useSpatialSupports, points = {{39, -40}, {12, -40}}, color = {0, 0, 127}));
  spinAxisWorld = Modelica.Mechanics.MultiBody.Frames.resolve1(uprightFrame.R, {0, 1, 0});
  articulationSpeed = if accountForCarrierMotion then spinAxisWorld*(Modelica.Mechanics.MultiBody.Frames.angularVelocity1(uprightFrame.R)-Modelica.Mechanics.MultiBody.Frames.angularVelocity1(sprungFrame.R))
    else 0;
  der(articulationAngle) = articulationSpeed;
  inboard.phi = outboard.phi+articulationAngle;
  inboard.tau+outboard.tau = 0;
  connect(
    sprungFrame,
    reaction.frame_a) annotation(Line(visible = not useSpatialSupports, points = {{-100, -40}, {-10, -40}}, color = {95, 95, 95}));
  connect(
    reaction.frame_b,
    uprightFrame) annotation(Line(visible = not useSpatialSupports, points = {{10, -40}, {100, -40}}, color = {95, 95, 95}));
  connect(
    reactionCommand.y,
    reaction.torque) annotation(Line(visible = not useSpatialSupports, points = {{-29, 0}, {-6, 0}, {-6, -28}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
    Line(points = {{-100, 60}, {100, 60}}, thickness = 4),
    Line(points = {{-100, -40}, {-40, -40}, {0, 20}, {40, -40}, {100, -40}}, color = {95, 95, 95}),
    Text(extent = {{-220, 120}, {220, 100}}, textString = "%name", textColor = {0, 0, 255})}),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 80}})),
    Documentation(info = "<html><p>useSpatialSupports selects the massless ideal shaft used with spatial rotors and explicit actuator supports. Only the torque-axis redirection is then carried by the sprung frame; there is no second upright-to-chassis brake couple. Power includes each rotor carrier angular velocity. All joint redirection reactions are lumped at the sprung frame; CV-joint force distribution and shaft bending are outside this model.</p><p>For BobLib's lumped wheel model, contact-patch moments already enter the upright. This ideal shaft transfers the reaction associated with its measured torque to the sprung structure using an equal and opposite couple; it does not add a second net external moment. The couple follows the wheel spin axis, the local y axis of uprightFrame.</p><p>This is a lumped reaction-path model with an ideal 1:1 scalar shaft. Relative carrier rotation about the transported wheel axis is included in shaft speed so the transfer conserves power. It does not represent CV-joint geometry, shaft bending, or shaft gyroscopic effects. Detailed CV forces and shaft bending require a dedicated joint/shaft model. Static four-post sweeps may combine explicit spatial supports with disabled carrier-speed coupling: their wheel rotation is prescribed and only the load transfer is evaluated.</p></html>"));
end InboardShaft;
