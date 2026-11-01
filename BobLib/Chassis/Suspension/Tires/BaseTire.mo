within BobLib.Chassis.Suspension.Tires;

model BaseTire

  import Modelica.Units.SI;
  import Modelica.Math.Vectors.normalize;

  extends BobLib.Icons.BaseTireIcon;

  // Record parameters
  parameter BobLib.Records.VehicleRecord.Chassis.Suspension.TireRecord pTire "Wheel geometry, compliance and tire-force calibration" annotation(Dialog(group = "Calibration"));
  parameter Boolean useWheelCompliance = false "Enable six-axis wheel-stack elasticity after the caliper mount" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Boolean quasiStatic = false "Disable wheel inertia for equilibrium rigs; also select elastic ComplianceModel" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Boolean useRoadSurface = false "Use the shared VehicleInterfaces road normal and friction in tire physics" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean includeRotorEffects = false
    "Include rotor gyroscopic and carrier-acceleration effects" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean exposeWheelBearing = false "Expose the non-spinning wheel-axis bearing frame" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Real referenceRoadFriction(min = Modelica.Constants.small) = 0.5
    "Road coefficient at which MF52 data were calibrated; forces scale by mu/referenceRoadFriction" annotation(Dialog(group = "Operating settings"));
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";
  parameter Boolean flatRoadFastPath = false "Use the horizontal z=0 road specialization; requires a compatible VI road" annotation(Dialog(group = "Advanced"), choices(checkBox = true), Evaluate = true);
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b bearingFrame if exposeWheelBearing annotation(
    Placement(transformation(origin = {120, 80}, extent = {{-12, -12}, {12, 12}}),
      iconTransformation(origin = {0, 100}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b unsprungFrame
    "Mass attachment downstream of wheel compliance; caliper reactions use bearingFrame" annotation(
    Placement(transformation(origin = {-120, 110}, extent = {{-12, -12}, {12, 12}}),
      iconTransformation(origin = {-60, 100}, extent = {{-12, -12}, {12, 12}})));

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a cpFrame annotation(
    Placement(
      transformation(origin = {0, -120}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame annotation(
    Placement(
      transformation(origin = {-120, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b hubFlange annotation(
    Placement(
      transformation(origin = {120, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  replaceable model ComplianceModel = BobLib.Chassis.Suspension.Tires.Wheels.DampedWheelCompliance constrainedby BobLib.Chassis.Suspension.Tires.Wheels.PartialWheelCompliance "Wheel-stack law; rigs redeclare the elastic-only variant" annotation(choicesAllMatching = true);

  // Base states
  SI.Force Fz;
  SI.Angle gamma;

  // Slip states (from SlipModel)
  SI.Angle alpha;
  Real kappa;

  replaceable BobLib.Chassis.Suspension.Tires.Wheels.Wheel0DOF wheelModel constrainedby BobLib.Chassis.Suspension.Tires.Wheels.PartialWheel(
    pWheel = pTire.wheel,
    redeclare model ComplianceModel = ComplianceModel,
    final useWheelCompliance = useWheelCompliance,
    final quasiStatic = quasiStatic,
    final exposeWheelBearing = exposeWheelBearing,
    final includeRotorEffects = includeRotorEffects) annotation(choicesAllMatching = true,
    Placement(transformation(extent = {{-30, -30}, {30, 30}})));
  replaceable BobLib.Chassis.Suspension.Tires.MF52.SlipModel.KinematicSlip slipModel constrainedby BobLib.Chassis.Suspension.Tires.MF52.SlipModel.BaseSlipModel annotation(choicesAllMatching = true,
    Placement(transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}})));

  // Force expressions
  Modelica.Blocks.Sources.RealExpression realExpressionFx(y = 0) annotation(
    Placement(transformation(origin = {-53.5, -52}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionFy(y = 0) annotation(
    Placement(transformation(origin = {-53.5, -80}, extent = {{-10, -10}, {10, 10}})));

  // Torque expressions
  Modelica.Blocks.Sources.RealExpression realExpressionMx(y = 0) annotation(
    Placement(transformation(origin = {-53.5, 84}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionMy(y = 0) annotation(
    Placement(transformation(origin = {-53.5, 56}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression realExpressionMz(y = 0) annotation(
    Placement(transformation(origin = {-53.5, 28}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world,
    animation = not headless) annotation(
    Placement(transformation(origin = {30, -50}, extent = {{-10, -10}, {10, 10}})));

protected
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadSurface surface(
    final flatRoadFastPath = flatRoadFastPath,
    r = cpFrame.r_0) if useRoadSurface
    annotation(Placement(visible = useRoadSurface, transformation(origin = {65, 95}, extent = {{-20, -12}, {20, 12}})));
  Modelica.Blocks.Interfaces.RealInput roadNormal[3] annotation(Placement(visible = useRoadSurface, transformation(origin = {110, 100}, extent = {{-3, -3}, {3, 3}})));
  Modelica.Blocks.Interfaces.RealInput roadMu annotation(Placement(visible = useRoadSurface, transformation(origin = {110, 65}, extent = {{-3, -3}, {3, 3}})));
  Real frictionScale;
  SI.Force contactForceWorld[3]
    "Contact-patch cut force resolved in the world frame";
  SI.Force tireForceWorld[3]
    "Tire shear force resolved in the road plane";
  SI.Torque tireTorqueWorld[3]
    "Tire moment resolved about the road-plane axes";
  Real[3] e_xw "Unit vector along wheel x-axis, resolved in world frame";
  Real[3] e_yw "Unit vector along wheel y-axis, resolved in world frame";
  Real[3] e_spin "Unit vector along wheel spin axis, resolved in world frame";
  Real[3] e_xg "Wheel heading projected onto the local road tangent plane";
  Real[3] e_yg "Road-plane lateral axis orthogonal to e_xg";

  // Velocity quantities (for slip calculation)
  SI.Velocity[3] v_cp "Contact patch velocity in world frame";
  SI.Velocity[3] v_g "Velocity projected onto ground plane";
  SI.Velocity Vx "Longitudinal velocity at contact patch";
  SI.Velocity Vy "Lateral velocity at contact patch";

equation
  connect(unsprungFrame, wheelModel.unsprungFrame) annotation(
    Line(points = {{-120, 110}, {-18, 110}, {-18, 30}}, color = {95, 95, 95}));
  connect(bearingFrame, wheelModel.bearingFrame) annotation(Line(visible = exposeWheelBearing,
    points = {{120, 80}, {108, 80}, {108, 18}, {30, 18}}, color = {95, 95, 95}));

  connect(
    surface.normal,
    roadNormal) annotation(Line(visible = useRoadSurface, points = {{85, 101}, {100, 101}, {100, 100}, {110, 100}}, color = {0, 0, 127}));
  connect(
    surface.mu,
    roadMu) annotation(Line(visible = useRoadSurface, points = {{85, 89}, {100, 89}, {100, 65}, {110, 65}}, color = {0, 0, 127}));

  if not useRoadSurface then
    roadNormal = {0, 0, 1};
    roadMu = referenceRoadFriction;
  end if;
  frictionScale = roadMu/referenceRoadFriction;
  assert(referenceRoadFriction>0, "Tire referenceRoadFriction must be positive");

  // Normal load
  contactForceWorld =
    Modelica.Mechanics.MultiBody.Frames.resolve1(cpFrame.R, cpFrame.f);
  Fz = noEvent(max(0, contactForceWorld*roadNormal));

  // World basis (from cpFrame)
  e_xw = Modelica.Mechanics.MultiBody.Frames.resolve1(cpFrame.R, {1, 0, 0});
  e_yw = Modelica.Mechanics.MultiBody.Frames.resolve1(cpFrame.R, {0, 1, 0});
  e_spin = Modelica.Mechanics.MultiBody.Frames.resolve1(wheelModel.hubAxis.frame_b.R, {0, 1, 0});

  assert(sum((e_xw-(e_xw*roadNormal)*roadNormal).^2)>1e-8,
    "Wheel heading is normal to road; point-contact tire basis is singular");

  // Ground-projected tire basis
  e_xg = normalize(e_xw-(e_xw*roadNormal)*roadNormal);
  e_yg = cross(roadNormal, e_xg);

  // Inclination angle
  gamma = Modelica.Math.asin(noEvent(max(-1.0, min(1.0, e_spin*roadNormal))));

  // Contact patch velocity
  v_cp = wheelModel.wheelVelSensor.v;
  v_g = v_cp-(v_cp*roadNormal)*roadNormal;
  Vx = v_g*e_xg;
  Vy = v_g*e_yg;

  // MF52 shear forces are defined in the road plane, using the same
  // ground-projected axes as the slip kinematics.  Tire moments follow the
  // corresponding road axes, with aligning moment about the road normal.
  tireForceWorld =
    frictionScale*(realExpressionFx.y*e_xg +
    realExpressionFy.y*e_yg);
  tireTorqueWorld =
    realExpressionMx.y*e_xg +
    realExpressionMy.y*e_yg +
    frictionScale*realExpressionMz.y*roadNormal;
  forceAndTorque.force = tireForceWorld;
  forceAndTorque.torque = tireTorqueWorld;

  // Slip model
  slipModel.Vx = Vx;
  slipModel.Vy = Vy;
  slipModel.omega = wheelModel.absoluteSpinSpeed;
  slipModel.R0 = wheelModel.radiusSensor.s_rel;
  slipModel.Fz = Fz;
  slipModel.gamma = gamma;
  alpha = slipModel.alpha;
  kappa = slipModel.kappa;

  connect(chassisFrame, wheelModel.chassisFrame) annotation(
    Line(points = {{-120, 0}, {-30, 0}}));
  connect(wheelModel.cpFrame, cpFrame) annotation(
    Line(points = {{0, -30}, {0, -120}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, wheelModel.cpFrame) annotation(
    Line(points = {{40, -50}, {45, -50}, {45, -34}, {0, -34}, {0, -30}}, color = {95, 95, 95}));
  connect(wheelModel.hubFlange, hubFlange) annotation(
    Line(points = {{0, 0}, {120, 0}}));
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
Standalone tire assemblies must provide an <code>inner VehicleInterfaces.Roads.FlatRoad road</code>
(or another VI road implementation). OpenModelica resolves this outer environment
even when the conditional road projection is disabled.</p><p>
With useRoadSurface, normal load, camber, slip axes and force directions use the
local VI road normal. Fx, Fy and aligning Mz scale by road.mu/referenceRoadFriction;
rolling resistance and overturning moments retain their calibration. The default
reference 0.5 preserves the previous calibration with VI FlatRoad's default mu.
This is a relative surface-condition multiplier, not a claim that the MF52 peak
friction equals road.mu. Wheels retain their existing radial point-contact geometry;
large-curvature circular-wheel contact requires a different wheel model.
</p>
</html>"));
end BaseTire;
