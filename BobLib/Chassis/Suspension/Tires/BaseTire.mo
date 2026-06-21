within BobLib.Chassis.Suspension.Tires;

model BaseTire

  extends BobLib.Icons.BaseTireIcon;

  import SI = Modelica.Units.SI;

  import Modelica.Math.Vectors.normalize;

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;

  // Record parameters
  parameter PartialWheelRecord pPartialWheel;
  outer parameter Boolean headless;

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a cpFrame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b hubFlange annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  // Base states
  SI.Force Fz;
  SI.Angle gamma;

  // Slip states (from SlipModel)
  SI.Angle alpha;
  Real kappa;

  replaceable BobLib.Chassis.Suspension.Tires.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pPartialWheel) annotation(
    Placement(transformation(extent = {{-30, -30}, {30, 30}})));
  replaceable BobLib.Chassis.Suspension.Tires.MF52.SlipModel.KinematicSlip slipModel annotation(
    Placement(transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}})));

  // Force expressions
  Modelica.Blocks.Sources.RealExpression realExpressionFx(y = 0) annotation(
    Placement(transformation(origin = {-90, -56}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionFy(y = 0) annotation(
    Placement(transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant constantZero(k = 0) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  // Torque expressions
  Modelica.Blocks.Sources.RealExpression realExpressionMx(y = 0) annotation(
    Placement(transformation(origin = {-90, 54}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionMy(y = 0) annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionMz(y = 0) annotation(
    Placement(transformation(origin = {-90, 26}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b,
    animation = not headless) annotation(
    Placement(transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}})));

protected
  Real[3] e_xw "Unit vector along wheel x-axis, resolved in world frame";
  Real[3] e_yw "Unit vector along wheel y-axis, resolved in world frame";
  Real[3] e_spin "Unit vector along wheel spin axis, resolved in world frame";
  Real[3] e_zw "Unit vector along wheel z-axis, resolved in world frame";
  Real[3] e_xg "e_xw projected on the xy-plane (ground) and normalized";
  Real[3] e_yg "e_yw projected on the xy-plane (ground) and normalized";

  // Velocity quantities (for slip calculation)
  Real[3] v_cp "Contact patch velocity in world frame";
  Real[3] v_g "Velocity projected onto ground plane";
  SI.Velocity Vx "Longitudinal velocity at contact patch";
  SI.Velocity Vy "Lateral velocity at contact patch";

equation

  // Normal load
  Fz = noEvent(max(0, cpFrame.f[3]));

  // World basis (from cpFrame)
  e_xw = Modelica.Mechanics.MultiBody.Frames.resolve1(cpFrame.R, {1, 0, 0});
  e_yw = Modelica.Mechanics.MultiBody.Frames.resolve1(cpFrame.R, {0, 1, 0});
  e_spin = Modelica.Mechanics.MultiBody.Frames.resolve1(wheelModel.hubAxis.frame_b.R, {0, 1, 0});
  e_zw = wheelModel.hubAxis.frame_b.R.T[:, 3];

  // Ground-projected tire basis
  e_xg = normalize({e_xw[1], e_xw[2], 0});
  e_yg = normalize({e_yw[1], e_yw[2], 0});

  // Inclination angle
  gamma = Modelica.Math.asin(noEvent(max(-1.0, min(1.0, e_zw[2]))));

  // Contact patch velocity
  v_cp = wheelModel.wheelVelSensor.v;
  v_g = {v_cp[1], v_cp[2], 0};
  Vx = v_g[1]*e_xg[1] + v_g[2]*e_xg[2];
  Vy = v_g[1]*e_yg[1] + v_g[2]*e_yg[2];

  // Slip model
  slipModel.Vx = Vx;
  slipModel.Vy = Vy;
  slipModel.omega = wheelModel.wheelRotSpeedSensor.w;
  slipModel.R0 = wheelModel.radiusSensor.s_rel;
  slipModel.Fz = Fz;
  slipModel.gamma = gamma;
  alpha = slipModel.alpha;
  kappa = slipModel.kappa;

  connect(chassisFrame, wheelModel.chassisFrame) annotation(
    Line(points = {{-100, 0}, {-30, 0}}));
  connect(wheelModel.cpFrame, cpFrame) annotation(
    Line(points = {{0, -30}, {0, -100}}, color = {95, 95, 95}));
  connect(realExpressionFx.y, forceAndTorque.force[1]) annotation(
    Line(points = {{-78, -56}, {-42, -56}}, color = {0, 0, 127}));
  connect(realExpressionFy.y, forceAndTorque.force[2]) annotation(
    Line(points = {{-78, -70}, {-60, -70}, {-60, -56}, {-42, -56}}, color = {0, 0, 127}));
  connect(constantZero.y, forceAndTorque.force[3]) annotation(
    Line(points = {{-78, -90}, {-50, -90}, {-50, -56}, {-42, -56}}, color = {0, 0, 127}));
  connect(realExpressionMx.y, forceAndTorque.torque[1]) annotation(
    Line(points = {{-78, 54}, {-50, 54}, {-50, -44}, {-42, -44}}, color = {0, 0, 127}));
  connect(realExpressionMy.y, forceAndTorque.torque[2]) annotation(
    Line(points = {{-78, 40}, {-60, 40}, {-60, -44}, {-42, -44}}, color = {0, 0, 127}));
  connect(realExpressionMz.y, forceAndTorque.torque[3]) annotation(
    Line(points = {{-78, 26}, {-70, 26}, {-70, -44}, {-42, -44}}, color = {0, 0, 127}));
  connect(forceAndTorque.frame_b, wheelModel.cpFrame) annotation(
    Line(points = {{-20, -50}, {0, -50}, {0, -30}}, color = {95, 95, 95}));
  connect(wheelModel.hubFlange, hubFlange) annotation(
    Line(points = {{0, 0}, {100, 0}}));
  annotation(Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Icon(graphics = {
      Line(origin = {50, 0}, points = {{-10, 0}, {50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {-50, 0}, points = {{-50, 0}, {10, 0}}),
      Line(origin = {0, -90}, points = {{0, -10}, {0, 10}})
    }),
    Documentation(info = "<html>
<p>
Model <code>BaseTire</code> assembles the common tire loop under the suspension domain.
</p>
<p>
It combines wheel physics, slip-state calculation, and force/torque application
while exposing wheel and raw contact-patch frames to the axle. Chassis-level
assemblies connect those frames to shared MultiBody contact mechanics.
</p>
</html>"));
end BaseTire;
