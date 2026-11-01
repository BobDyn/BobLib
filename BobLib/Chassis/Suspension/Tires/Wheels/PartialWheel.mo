within BobLib.Chassis.Suspension.Tires.Wheels;

partial model PartialWheel

  parameter BobLib.Records.VehicleRecord.Chassis.Suspension.WheelRecord pWheel "Wheel geometry, rotational inertia and vertical compliance calibration" annotation(Dialog(group = "Calibration"));

  extends BobLib.Icons.PartialWheelIcon;

  parameter Boolean includeRotorEffects = false
    "Include rotor gyroscopic and carrier-acceleration effects" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  parameter Boolean useWheelCompliance = false "Enable six-axis elasticity downstream of the upright/caliper support" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Boolean quasiStatic = false "Disable wheel inertia for equilibrium rigs; also select elastic ComplianceModel" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));

protected
  parameter Modelica.Units.SI.Inertia wheelJ = pWheel.rotation.wheelJ "Effective inertia for this wheel architecture";

public
  outer Modelica.Mechanics.MultiBody.World world;

  // Record parameters

  parameter Boolean exposeWheelBearing = false "Expose the non-spinning wheel-axis bearing frame" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Real longitudinalTorqueSign = -1
    "Sign mapping contact-patch longitudinal force to wheel-axis torque" annotation(Dialog(group = "Operating settings"));
  outer parameter Modelica.Units.SI.Length linkDiameter "Diameter of links in the MultiBody animation; does not set structural stiffness";
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b bearingFrame if exposeWheelBearing annotation(
    Placement(transformation(origin = {-120, 100}, extent = {{-12, -12}, {12, 12}}),
      iconTransformation(origin = {100, 60}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b unsprungFrame
    "Non-spinning mass attachment after compliance, in the upright calibration axes at zero deflection" annotation(
    Placement(transformation(origin = {120, -100}, extent = {{-12, -12}, {12, 12}}),
      iconTransformation(origin = {-60, 100}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Mechanics.MultiBody.Parts.FixedRotation massOrientation(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence = {3, 2, 1},
    angles = {-pWheel.geometry.staticAlpha, 0, -pWheel.geometry.staticGamma},
    animation = false) "Undo static wheel alignment for the existing unsprung mass calibration" annotation(
    Placement(transformation(origin = {70, -90}, extent = {{-10, -10}, {10, 10}})));

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a cpFrame annotation(
    Placement(
      transformation(origin = {0, -120}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame annotation(
    Placement(
      transformation(origin = {-120, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b hubFlange annotation(
    Placement(
      transformation(origin = {-80, 120}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(extent = {{-10, -10}, {10, 10}})));

  output Modelica.Units.SI.AngularVelocity absoluteSpinSpeed
    "Wheel angular velocity about its spin axis in the inertial frame";

  replaceable model ComplianceModel = DampedWheelCompliance constrainedby PartialWheelCompliance "Wheel-stack law; equilibrium rigs select WheelCompliance" annotation(choicesAllMatching = true);

  ComplianceModel compliance(pCompliance = pWheel.compliance) if useWheelCompliance annotation(Placement(visible = useWheelCompliance, transformation(origin = {-15, 0}, extent = {{-10, -10}, {10, 10}})));

  // Tire attitude
  Modelica.Mechanics.MultiBody.Parts.FixedRotation toHub(rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
                                                         sequence = {1, 2, 3}, angles = {pWheel.geometry.staticGamma, 0, pWheel.geometry.staticAlpha}) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));

  // Rotational physics (wheel rotation about the hub axis)
  Modelica.Mechanics.MultiBody.Joints.Revolute hubAxis(
    n = {0, 1, 0},
    useAxisFlange = true,
    animation = not headless,
    phi(start = 0, fixed = true, nominal = 0.1),
    w(start = 0, nominal = 100)) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));

  // Translational physics (wheel vertical deflection)
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_z(
    useAxisFlange = true,
    n = {0, 0, -1},
    animation = not headless,
    boxWidth = linkDiameter,
    boxHeight = linkDiameter) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));

  // Rotational dynamics
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J = if quasiStatic then 0 else wheelJ) if not includeRotorEffects annotation(
    Placement(visible = not includeRotorEffects, transformation(origin = {40, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Torque tireTorqueSource(useSupport = true) annotation(
    Placement(transformation(origin = {30, 60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D spatialInertia(
    J = if quasiStatic then 0 else wheelJ,
    n = {0, 1, 0},
    animation = false)
    if includeRotorEffects annotation(
    Placement(visible = includeRotorEffects, transformation(origin = {40, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D roadReaction(n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}})));

  // Torque generation
  Modelica.Blocks.Sources.RealExpression tireTorque(
    y = longitudinalTorqueSign*cpFrame.f[1]*radiusSensor.s_rel) annotation(
    Placement(transformation(origin = {-20, 60}, extent = {{-10, -10}, {10, 10}})));

  // Sensors
  Modelica.Mechanics.Translational.Sensors.RelPositionSensor radiusSensor annotation(
    Placement(transformation(origin = {-80, -65}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelRotSpeedSensor annotation(
    Placement(transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity wheelVelSensor(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{-10, -10}, {10, 10}})));

  // Visualizers
  Modelica.Mechanics.MultiBody.Visualizers.VoluminousWheel voluminousWheel(
    rRim = pWheel.geometry.rimR0,
    rTire = pWheel.geometry.R0,
    width = pWheel.geometry.rimWidth,
    animation = not headless) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

equation
  connect(hubAxis.frame_a, massOrientation.frame_a) annotation(
    Line(points = {{10, 0}, {0, 0}, {0, -16}, {42, -16}, {42, -90}, {60, -90}}, color = {95, 95, 95}));
  connect(massOrientation.frame_b, unsprungFrame) annotation(
    Line(points = {{80, -90}, {100, -90}, {100, -100}, {120, -100}}, color = {95, 95, 95}));
  absoluteSpinSpeed = wheelRotSpeedSensor.w +
    Modelica.Mechanics.MultiBody.Frames.angularVelocity2(hubAxis.frame_a.R)*{0, 1, 0};
  assert(not includeRotorEffects or world.driveTrainMechanics3D,
    "Spatial wheel requires world.driveTrainMechanics3D=true");
  connect(tireTorqueSource.support, roadReaction.flange_b) annotation(
    Line(points = {{30, 70}, {110, 70}, {110, 90}, {100, 90}}, color = {0, 0, 0}));
  connect(roadReaction.frame_a, hubAxis.frame_a) annotation(
    Line(
      visible = world.driveTrainMechanics3D,
      points = {{90, 80}, {114, 80}, {114, -16}, {0, -16}, {0, 0}, {10, 0}},
      color = {95, 95, 95}));
  connect(spatialInertia.frame_a, hubAxis.frame_a) annotation(
    Line(
      visible = includeRotorEffects,
      points = {{40, 20}, {40, 14}, {0, 14}, {0, 0}, {10, 0}},
      color = {95, 95, 95}));
  connect(bearingFrame, toHub.frame_b) annotation(Line(visible = exposeWheelBearing,
    points = {{-120, 100}, {-108, 100}, {-108, 20}, {-40, 20}, {-40, 0}}, color = {95, 95, 95}));
  connect(hubAxis.axis, inertia.flange_a) annotation(
    Line(visible = not includeRotorEffects, points = {{20, 10}, {20, 30}, {30, 30}}));
  connect(hubAxis.axis, spatialInertia.flange_a) annotation(
    Line(visible = includeRotorEffects, points = {{20, 10}, {20, 30}, {30, 30}}));
  connect(prismatic_z.frame_a, hubAxis.frame_a) annotation(
    Line(points = {{0, -30}, {0, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(prismatic_z.frame_b, cpFrame) annotation(
    Line(points = {{0, -50}, {0, -120}}, color = {95, 95, 95}));
  connect(radiusSensor.flange_a, prismatic_z.support) annotation(
    Line(points = {{-80, -55}, {-80, -30}, {-6, -30}, {-6, -36}}, color = {0, 127, 0}));
  connect(radiusSensor.flange_b, prismatic_z.axis) annotation(
    Line(points = {{-80, -75}, {-80, -85}, {-6, -85}, {-6, -48}}, color = {0, 127, 0}));
  connect(tireTorque.y, tireTorqueSource.tau) annotation(
    Line(points = {{-9, 60}, {18, 60}}, color = {0, 0, 127}));
  connect(voluminousWheel.frame_a, hubAxis.frame_b) annotation(
    Line(points = {{60, 0}, {30, 0}}, color = {95, 95, 95}));
  connect(wheelVelSensor.frame_a, hubAxis.frame_a) annotation(
    Line(points = {{60, -50}, {42, -50}, {42, -16}, {0, -16}, {0, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(tireTorqueSource.flange, inertia.flange_b) annotation(
    Line(visible = not includeRotorEffects, points = {{40, 60}, {60, 60}, {60, 30}, {50, 30}}));
  connect(tireTorqueSource.flange, spatialInertia.flange_b) annotation(
    Line(visible = includeRotorEffects, points = {{40, 60}, {60, 60}, {60, 30}, {50, 30}}));
  connect(wheelRotSpeedSensor.flange, inertia.flange_b) annotation(
    Line(visible = not includeRotorEffects, points = {{80, 50}, {60, 50}, {60, 30}, {50, 30}}));
  connect(wheelRotSpeedSensor.flange, spatialInertia.flange_b) annotation(
    Line(visible = includeRotorEffects, points = {{80, 50}, {60, 50}, {60, 30}, {50, 30}}));
  connect(hubFlange, inertia.flange_a) annotation(
    Line(visible = not includeRotorEffects, points = {{-80, 120}, {-80, 40}, {20, 40}, {20, 30}, {30, 30}}));
  connect(hubFlange, spatialInertia.flange_a) annotation(
    Line(visible = includeRotorEffects, points = {{-80, 120}, {-80, 40}, {20, 40}, {20, 30}, {30, 30}}));
  connect(chassisFrame, toHub.frame_a) annotation(
    Line(points = {{-120, 0}, {-60, 0}}));

  if not useWheelCompliance then
    connect(
      toHub.frame_b,
      hubAxis.frame_a) annotation(Line(visible = not useWheelCompliance, points = {{-40, 0}, {10, 0}}, color = {95, 95, 95}));
  end if;
  connect(
    toHub.frame_b,
    compliance.frame_a) annotation(Line(visible = useWheelCompliance, points = {{-40, 0}, {-25, 0}}, color = {95, 95, 95}));
  connect(
    compliance.frame_b,
    hubAxis.frame_a) annotation(Line(visible = useWheelCompliance, points = {{-5, 0}, {10, 0}}, color = {95, 95, 95}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Icon(graphics = {
      Line(origin = {-63, 0}, points = {{-37, 0}, {63, 0}}, thickness = 5),
      Line(origin = {0, -86}, points = {{0, -14}, {0, 24}}, color = {0, 170, 0}, thickness = 5)
    }),
    Documentation(info = "<html>
<p>
Partial model <code>PartialWheel</code> defines shared wheel-body frames, geometry, and outputs.
</p>
<p>The unsprungFrame attaches the existing unsprung mass downstream of wheel-stack compliance and before wheel spin. massOrientation reverses the fixed camber/toe transform so mass-record CG and inertia retain their upright calibration axes at zero compliance deflection. The full unsprung assembly is lumped downstream; the caliper support remains upstream. No mass is added here.</p>
<p>Road-torque conversion is supported at the upright. includeRotorEffects
independently replaces scalar spin inertia with Rotor1D for gyroscopic and
carrier-acceleration corrections. The unsprung mass record must include the
complete locked-wheel inertia tensor. Do not subtract wheelJ from that tensor:
the mass is attached before hubAxis rotation, and Rotor1D supplies the
carrier-acceleration correction rather than another complete rigid body.
Only one of scalar inertia and Rotor1D is instantiated. With a stationary
carrier, shaft torque is wheelJ times spin acceleration; with locked relative
spin, the combined carrier and shaft torque uses the locked tensor exactly
once. Scalar inertia omits the spatial carrier correction by design.
In the vehicle, brakes provide explicit
caliper supports; do not also apply a legacy upright-to-chassis reaction couple.</p>
<p>
Concrete wheel models extend it to choose whether lateral, vertical, and rotational degrees of freedom are fixed or dynamic.
</p>
</html>"));
end PartialWheel;
