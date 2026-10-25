within BobLib.Chassis.Suspension.BaseClasses;

partial model PartialDoubleWishboneAxle

  import Modelica.Units.SI;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.AxleMassRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoopRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;
  import BobLib.Utilities.Math.Tensor;
  import BobLib.Utilities.Math.Vector;

  replaceable record AxleRecord = BobLib.Records.VehicleRecord.Chassis.Suspension.DoubleWishboneBellcrankStabarRecord
    constrainedby BobLib.Records.VehicleRecord.Chassis.Suspension.AxleRecord;

  parameter AxleRecord pAxle "Axle calibration (left half; the right half is mirrored)" annotation(Dialog(group = "Calibration"));

  extends BobLib.Chassis.Suspension.Interfaces.AxleBase(
    leftContactTorque = leftTire.wheelModel.tireTorqueSource.tau,
    rightContactTorque = rightTire.wheelModel.tireTorqueSource.tau,
    final showBaseIcon = false);
  extends BobLib.Icons.AxleDWBaseIcon;

  extends BobLib.Icons.SteeringWheelOverlayIcon(final showSteering = steerable);

  parameter Modelica.Units.SI.Position lockedRackPosition = 0
    "Rack displacement when steering is disabled" annotation(Dialog(group = "Operating settings", enable = not steerable));

  parameter Boolean useWheelCompliance = false "Enable the same wheel-stack compliance architecture on both sides" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter Boolean quasiStatic = false "Omit body inertia and damping for equilibrium rigs; retain elastic calibration" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));

  // Record parameters

  parameter Boolean fixInitialLeftLowerArmAngle = false
    "Use the supplied left lower-arm angle as a fixed initial suspension state" annotation(Dialog(group = "Initialization"), choices(checkBox = true));
  parameter Boolean fixInitialRightLowerArmAngle = false
    "Use the supplied right lower-arm angle as a fixed initial suspension state" annotation(Dialog(group = "Initialization"), choices(checkBox = true));
  parameter Boolean fixInitialLowerArmAngularVelocities = false
    "Initialize both lower-arm angular velocities to zero" annotation(Dialog(group = "Initialization"), choices(checkBox = true));
  parameter SI.Angle initialLeftLowerArmAngle = 0
    "Four-post-solved initial left lower-arm angle" annotation(Dialog(group = "Initialization"));
  parameter SI.Angle initialRightLowerArmAngle = 0
    "Four-post-solved initial right lower-arm angle" annotation(Dialog(group = "Initialization"));

  final parameter PartialWheelRecord pRightPartialWheel(R0 = pAxle.tire.wheel.geometry.R0,
                                                  rimR0 = pAxle.tire.wheel.geometry.rimR0,
                                                  rimWidth = pAxle.tire.wheel.geometry.rimWidth,
                                                  staticAlpha = -pAxle.tire.wheel.geometry.staticAlpha,
                                                  staticGamma = -pAxle.tire.wheel.geometry.staticGamma) "Right wheel geometry obtained by mirroring the left wheel alignment";

  final parameter WishboneUprightLoopRecord pRightDW(upperFore_i = Vector.mirrorXZ(pAxle.geometry.upperFore_i),
                                               upperAft_i = Vector.mirrorXZ(pAxle.geometry.upperAft_i),
                                               lowerFore_i = Vector.mirrorXZ(pAxle.geometry.lowerFore_i),
                                               lowerAft_i = Vector.mirrorXZ(pAxle.geometry.lowerAft_i),
                                               upper_o = Vector.mirrorXZ(pAxle.geometry.upper_o),
                                               lower_o = Vector.mirrorXZ(pAxle.geometry.lower_o),
                                               tie_o = Vector.mirrorXZ(pAxle.geometry.tie_o),
                                               wheelCenter = Vector.mirrorXZ(pAxle.geometry.wheelCenter)) "Right wishbone and upright hardpoints mirrored across the vehicle XZ plane";

  final parameter AxleMassRecord pRightAxleMass(unsprungMass(m = pAxle.masses.unsprungMass.m,
                                                       rCM = Vector.mirrorXZ(pAxle.masses.unsprungMass.rCM),
                                                       inertia = Tensor.mirrorXZ(pAxle.masses.unsprungMass.inertia)),
                                          ucaMass(m = pAxle.masses.ucaMass.m,
                                                  rCM = Vector.mirrorXZ(pAxle.masses.ucaMass.rCM),
                                                  inertia = Tensor.mirrorXZ(pAxle.masses.ucaMass.inertia)),
                                          lcaMass(m = pAxle.masses.lcaMass.m,
                                                  rCM = Vector.mirrorXZ(pAxle.masses.lcaMass.rCM),
                                                  inertia = Tensor.mirrorXZ(pAxle.masses.lcaMass.inertia)),
                                          tieMass(m = pAxle.masses.tieMass.m,
                                                  rCM = Vector.mirrorXZ(pAxle.masses.tieMass.rCM),
                                                  inertia = Tensor.mirrorXZ(pAxle.masses.tieMass.inertia))) "Right suspension mass properties mirrored across the vehicle XZ plane";

  // Visual parameters
  outer parameter SI.Length linkDiameter "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  outer parameter SI.Length jointDiameter "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";

  // Effective center for internal calculations
  final parameter SI.Position[3] effectiveCenter = {pAxle.geometry.wheelCenter[1], 0, pAxle.geometry.wheelCenter[3]}
    "Axle center reference at wheel-center X and Z, on the vehicle symmetry plane";

  Modelica.Blocks.Interfaces.RealOutput leftSpringLength(unit = "m") "Left spring/damper length";

  Modelica.Blocks.Interfaces.RealOutput rightSpringLength(unit = "m") "Right spring/damper length";
  SI.Angle stabarAngle "Stabilizer-bar angular deflection";

  final parameter BobLib.Records.VehicleRecord.Chassis.Suspension.TireRecord rightTireCalibration(
    wheel(
      geometry = pRightPartialWheel,
      rotation = pAxle.tire.wheel.rotation,
      vertical = pAxle.tire.wheel.vertical,
      compliance = pAxle.tire.wheel.compliance),
    force = pAxle.tire.force) "Right tire calibration with mirrored alignment and shared force and compliance data";

  // Tires
  replaceable BobLib.Chassis.Suspension.Tires.MF52Tire leftTire(
    redeclare replaceable BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheelModel(longitudinalTorqueSign = -1)) constrainedby BobLib.Chassis.Suspension.Tires.BaseTire(
    pTire = pAxle.tire,
    final quasiStatic = quasiStatic,
    useWheelCompliance = useWheelCompliance,
    final exposeWheelBearing = exposeWheelBearings,
    final includeRotorEffects = includeRotorEffects,
    final useRoadSurface = useRoadSurface,
    final flatRoadFastPath = flatRoadFastPath) annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-160, 50}, extent = {{10, -10}, {-10, 10}})));

  replaceable BobLib.Chassis.Suspension.Tires.MF52Tire rightTire(
    redeclare replaceable BobLib.Chassis.Suspension.Tires.Wheels.Wheel1DOF_Y wheelModel(longitudinalTorqueSign = -1)) constrainedby BobLib.Chassis.Suspension.Tires.BaseTire(
    pTire = rightTireCalibration,
    final quasiStatic = quasiStatic,
    useWheelCompliance = useWheelCompliance,
    final exposeWheelBearing = exposeWheelBearings,
    final includeRotorEffects = includeRotorEffects,
    final useRoadSurface = useRoadSurface,
    final flatRoadFastPath = flatRoadFastPath) annotation(choicesAllMatching = true,
    Placement(transformation(origin = {160, 50}, extent = {{-10, -10}, {10, 10}})));

  // Double wishbones
  BobLib.Chassis.Suspension.Components.WishboneUprightLoop leftWishboneUprightLoop(
    pDW = pAxle.geometry,
    final linkDiameter = linkDiameter,
    final jointDiameter = jointDiameter,
    lowerJoint_i(
      phi(start = initialLeftLowerArmAngle, fixed = fixInitialLeftLowerArmAngle),
      w(start = 0, fixed = fixInitialLowerArmAngularVelocities))) annotation(
    Placement(transformation(origin = {-69, 50}, extent = {{29, -29}, {-29, 29}})));

  BobLib.Chassis.Suspension.Components.WishboneUprightLoop rightWishboneUprightLoop(
    pDW = pRightDW,
    final linkDiameter = linkDiameter,
    final jointDiameter = jointDiameter,
    lowerJoint_i(
      phi(start = initialRightLowerArmAngle, fixed = fixInitialRightLowerArmAngle),
      w(start = 0, fixed = fixInitialLowerArmAngularVelocities))) annotation(
    Placement(transformation(origin = {69, 50}, extent = {{-29, -29}, {29, 29}})));

  BobLib.Chassis.Suspension.Components.Rod leftTieRod(
    pRod = pAxle.masses.tieMass,
    final useMass = not quasiStatic,
    r_a = pAxle.steering.leftPickup,
    r_b = pAxle.geometry.tie_o,
    show_universal_axes = false,
    kinematicConstraint = true,
    final linkDiameter = linkDiameter,
    final jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-60, 115}, extent = {{15, -15}, {-15, 15}})));

  BobLib.Chassis.Suspension.Components.Rod rightTieRod(
    pRod = pRightAxleMass.tieMass,
    final useMass = not quasiStatic,
    r_a = Vector.mirrorXZ(pAxle.steering.leftPickup),
    r_b = pRightDW.tie_o,
    show_universal_axes = false,
    kinematicConstraint = true,
    final linkDiameter = linkDiameter,
    final jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {60, 115}, extent = {{-15, -15}, {15, 15}})));

  BobLib.Chassis.Suspension.Components.RackAndPinion rackAndPinion(
    pRack = pAxle.steering,
    final steerable = steerable,
    final lockedRackPosition = lockedRackPosition,
    final linkDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {0, 110}, extent = {{-20, -20}, {20, 20}})));

protected
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRack(
    r = {pAxle.steering.leftPickup[1], 0, pAxle.steering.leftPickup[3]} - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {0, 82}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation leftTieConnection(
    r = pAxle.geometry.lower_o - pAxle.geometry.tie_o,
    animation = false) annotation(
    Placement(transformation(origin = {-115, 65}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightTieConnection(
    r = pRightDW.lower_o - pRightDW.tie_o,
    animation = false) annotation(
    Placement(transformation(origin = {115, 65}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftWheelCenter(
    r = pAxle.geometry.wheelCenter - pAxle.geometry.lower_o,
    animation = false) annotation(
    Placement(transformation(origin = {-120, 30}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightWheelCenter(
    r = pRightDW.wheelCenter - pRightDW.lower_o,
    animation = false) annotation(
    Placement(transformation(origin = {120, 30}, extent = {{-10, -10}, {10, 10}})));

  // Fixed geometry from effective center to nodes
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftUpper_i(
    r = (pAxle.geometry.upperFore_i + pAxle.geometry.upperAft_i)/2 - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftLower_i(
    r = (pAxle.geometry.lowerFore_i + pAxle.geometry.lowerAft_i)/2 - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, 30}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightUpper_i(
    r = (pRightDW.upperFore_i + pRightDW.upperAft_i)/2 - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightLower_i(
    r = (pRightDW.lowerFore_i + pRightDW.lowerAft_i)/2 - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, 30}, extent = {{-10, -10}, {10, 10}})));

  // Left-half bodies
  Modelica.Mechanics.MultiBody.Parts.Body leftUCABody(m = pAxle.masses.ucaMass.m,
                                                      r_CM = pAxle.masses.ucaMass.rCM - pAxle.geometry.upper_o,
                                                      I_11 = pAxle.masses.ucaMass.inertia[1, 1],
                                                      I_22 = pAxle.masses.ucaMass.inertia[2, 2],
                                                      I_33 = pAxle.masses.ucaMass.inertia[3, 3],
                                                      I_21 = pAxle.masses.ucaMass.inertia[2, 1],
                                                      I_31 = pAxle.masses.ucaMass.inertia[3, 1],
                                                      I_32 = pAxle.masses.ucaMass.inertia[3, 2],
                                                      sphereDiameter = jointDiameter,
                                                      cylinderDiameter = linkDiameter,
                                                      useQuaternions = false) if not quasiStatic annotation(
    Placement(visible = not quasiStatic, transformation(origin = {-105, 100}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body leftLCABody(m = pAxle.masses.lcaMass.m,
                                                      r_CM = pAxle.masses.lcaMass.rCM - pAxle.geometry.lower_o,
                                                      I_11 = pAxle.masses.lcaMass.inertia[1, 1],
                                                      I_22 = pAxle.masses.lcaMass.inertia[2, 2],
                                                      I_33 = pAxle.masses.lcaMass.inertia[3, 3],
                                                      I_21 = pAxle.masses.lcaMass.inertia[2, 1],
                                                      I_31 = pAxle.masses.lcaMass.inertia[3, 1],
                                                      I_32 = pAxle.masses.lcaMass.inertia[3, 2],
                                                      sphereDiameter = jointDiameter,
                                                      cylinderDiameter = linkDiameter,
                                                      useQuaternions = false) if not quasiStatic annotation(
    Placement(
      visible = not quasiStatic,
      transformation(origin = {-130, 8}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Parts.Body leftUnsprungBody(m = pAxle.masses.unsprungMass.m,
                                                           r_CM = pAxle.masses.unsprungMass.rCM - pAxle.geometry.wheelCenter,
                                                           I_11 = pAxle.masses.unsprungMass.inertia[1, 1],
                                                           I_22 = pAxle.masses.unsprungMass.inertia[2, 2],
                                                           I_33 = pAxle.masses.unsprungMass.inertia[3, 3],
                                                           I_21 = pAxle.masses.unsprungMass.inertia[2, 1],
                                                           I_31 = pAxle.masses.unsprungMass.inertia[3, 1],
                                                           I_32 = pAxle.masses.unsprungMass.inertia[3, 2],
                                                           sphereDiameter = jointDiameter,
                                                           cylinderDiameter = linkDiameter,
                                                           useQuaternions = false) if not quasiStatic annotation(
    Placement(visible = not quasiStatic, transformation(origin = {-145, 85}, extent = {{10, -10}, {-10, 10}})));

  // Right-half bodies
  Modelica.Mechanics.MultiBody.Parts.Body rightUCABody(m = pRightAxleMass.ucaMass.m,
                                                       r_CM = pRightAxleMass.ucaMass.rCM - pRightDW.upper_o,
                                                       I_11 = pRightAxleMass.ucaMass.inertia[1, 1],
                                                       I_22 = pRightAxleMass.ucaMass.inertia[2, 2],
                                                       I_33 = pRightAxleMass.ucaMass.inertia[3, 3],
                                                       I_21 = pRightAxleMass.ucaMass.inertia[2, 1],
                                                       I_31 = pRightAxleMass.ucaMass.inertia[3, 1],
                                                       I_32 = pRightAxleMass.ucaMass.inertia[3, 2],
                                                       sphereDiameter = jointDiameter,
                                                       cylinderDiameter = linkDiameter,
                                                       useQuaternions = false) if not quasiStatic annotation(
    Placement(
      visible = not quasiStatic,
      transformation(origin = {105, 100}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Parts.Body rightLCABody(m = pRightAxleMass.lcaMass.m,
                                                       r_CM = pRightAxleMass.lcaMass.rCM - pRightDW.lower_o,
                                                       I_11 = pRightAxleMass.lcaMass.inertia[1, 1],
                                                       I_22 = pRightAxleMass.lcaMass.inertia[2, 2],
                                                       I_33 = pRightAxleMass.lcaMass.inertia[3, 3],
                                                       I_21 = pRightAxleMass.lcaMass.inertia[2, 1],
                                                       I_31 = pRightAxleMass.lcaMass.inertia[3, 1],
                                                       I_32 = pRightAxleMass.lcaMass.inertia[3, 2],
                                                       sphereDiameter = jointDiameter,
                                                       cylinderDiameter = linkDiameter,
                                                       useQuaternions = false) if not quasiStatic annotation(
    Placement(visible = not quasiStatic, transformation(origin = {130, 8}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body rightUnsprungBody(m = pRightAxleMass.unsprungMass.m,
                                                            r_CM = pRightAxleMass.unsprungMass.rCM - pRightDW.wheelCenter,
                                                            I_11 = pRightAxleMass.unsprungMass.inertia[1, 1],
                                                            I_22 = pRightAxleMass.unsprungMass.inertia[2, 2],
                                                            I_33 = pRightAxleMass.unsprungMass.inertia[3, 3],
                                                            I_21 = pRightAxleMass.unsprungMass.inertia[2, 1],
                                                            I_31 = pRightAxleMass.unsprungMass.inertia[3, 1],
                                                            I_32 = pRightAxleMass.unsprungMass.inertia[3, 2],
                                                            sphereDiameter = jointDiameter,
                                                            cylinderDiameter = linkDiameter,
                                                            useQuaternions = false) if not quasiStatic annotation(
    Placement(visible = not quasiStatic, transformation(origin = {145, 85}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealOutput leftNormalLoad(unit = "N") = leftTire.Fz;

  Modelica.Blocks.Interfaces.RealOutput rightNormalLoad(unit = "N") = rightTire.Fz;

equation
  connect(leftBearing, leftTire.bearingFrame) annotation(Line(visible = exposeWheelBearings,
    points = {{-180, 110}, {-170, 110}, {-170, 64}, {-160, 64}, {-160, 60}}, color = {95, 95, 95}));
  connect(rightBearing, rightTire.bearingFrame) annotation(Line(visible = exposeWheelBearings,
    points = {{180, 110}, {170, 110}, {170, 64}, {160, 64}, {160, 60}}, color = {95, 95, 95}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(visible = steerable, points = {{0, 114}, {0, 140}}));
  connect(leftNormalLoad, axleBus.leftNormalLoad) annotation(Line(visible = false, points = {{180, 140}, {180, 140}}));
  connect(
    rightNormalLoad,
    axleBus.rightNormalLoad) annotation(Line(visible = false, points = {{180, 140}, {180, 140}}));
  connect(
    leftSpringLength,
    axleBus.leftSpringLength) annotation(Line(visible = false, points = {{180, 140}, {180, 140}}));
  connect(
    rightSpringLength,
    axleBus.rightSpringLength) annotation(Line(visible = false, points = {{180, 140}, {180, 140}}));
  connect(leftCP, leftTire.cpFrame) annotation(
    Line(points = {{-180, -70}, {-180, 36}, {-160, 36}, {-160, 40}}));
  connect(rightCP, rightTire.cpFrame) annotation(
    Line(points = {{180, -70}, {180, 36}, {160, 36}, {160, 40}}));
  connect(leftTorque, leftTire.hubFlange) annotation(
    Line(points = {{-180, 50}, {-170, 50}}));
  connect(rightTorque, rightTire.hubFlange) annotation(
    Line(points = {{180, 50}, {170, 50}}));
  connect(toLeftUpper_i.frame_a, toRack.frame_a) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, 76}}));
  connect(toLeftLower_i.frame_a, toRightLower_i.frame_a) annotation(
    Line(points = {{-10, 30}, {10, 30}}));
  connect(toLeftLower_i.frame_a, toLeftUpper_i.frame_a) annotation(
    Line(points = {{-10, 30}, {0, 30}, {0, 70}, {-10, 70}}));
  connect(toLeftUpper_i.frame_a, toRightUpper_i.frame_a) annotation(
    Line(points = {{-10, 70}, {10, 70}}));
  connect(toLeftLower_i.frame_b, leftWishboneUprightLoop.lowerFrame_i) annotation(
    Line(points = {{-30, 30}, {-33, 30}, {-33, 29.7}, {-40, 29.7}}, color = {95, 95, 95}));
  connect(toLeftUpper_i.frame_b, leftWishboneUprightLoop.upperFrame_i) annotation(
    Line(points = {{-30, 70}, {-33, 70}, {-33, 70.3}, {-40, 70.3}}, color = {95, 95, 95}));
  connect(toRightLower_i.frame_b, rightWishboneUprightLoop.lowerFrame_i) annotation(
    Line(points = {{30, 30}, {33, 30}, {33, 29.7}, {40, 29.7}}, color = {95, 95, 95}));
  connect(toRightUpper_i.frame_b, rightWishboneUprightLoop.upperFrame_i) annotation(
    Line(points = {{30, 70}, {33, 70}, {33, 70.3}, {40, 70.3}}, color = {95, 95, 95}));
  connect(axleFrame, toLeftLower_i.frame_a) annotation(
    Line(points = {{32, -180}, {32, -95}, {0, -95}, {0, 30}, {-10, 30}}));
  connect(toRack.frame_b, rackAndPinion.mountFrame) annotation(
    Line(points = {{0, 88}, {0, 106}}, color = {95, 95, 95}));
  connect(rackAndPinion.leftFrame, leftTieRod.frame_a) annotation(
    Line(points = {{-20, 110}, {-30, 110}, {-30, 115}, {-45, 115}}, color = {95, 95, 95}));
  connect(rackAndPinion.rightFrame, rightTieRod.frame_a) annotation(
    Line(points = {{20, 110}, {30, 110}, {30, 115}, {45, 115}}, color = {95, 95, 95}));
  connect(leftTieRod.frame_b, leftTieConnection.frame_a) annotation(
    Line(points = {{-75, 115}, {-125, 115}, {-125, 75}, {-115, 75}}, color = {95, 95, 95}));
  connect(rightTieRod.frame_b, rightTieConnection.frame_a) annotation(
    Line(points = {{75, 115}, {125, 115}, {125, 75}, {115, 75}}, color = {95, 95, 95}));
  connect(leftTieConnection.frame_b, leftWishboneUprightLoop.steeringFrame) annotation(
    Line(points = {{-115, 55}, {-105, 55}, {-105, 29.7}, {-98, 29.7}}, color = {95, 95, 95}));
  connect(rightTieConnection.frame_b, rightWishboneUprightLoop.steeringFrame) annotation(
    Line(points = {{115, 55}, {105, 55}, {105, 29.7}, {98, 29.7}}, color = {95, 95, 95}));
  connect(leftWishboneUprightLoop.steeringFrame, toLeftWheelCenter.frame_a) annotation(
    Line(points = {{-98, 29.7}, {-104, 29.7}, {-104, 30}, {-110, 30}}, color = {95, 95, 95}));
  connect(rightWishboneUprightLoop.steeringFrame, toRightWheelCenter.frame_a) annotation(
    Line(points = {{98, 29.7}, {104, 29.7}, {104, 30}, {110, 30}}, color = {95, 95, 95}));
  connect(leftTire.chassisFrame, toLeftWheelCenter.frame_b) annotation(
    Line(points = {{-150, 50}, {-145, 50}, {-145, 30}, {-130, 30}}, color = {95, 95, 95}));
  connect(rightTire.chassisFrame, toRightWheelCenter.frame_b) annotation(
    Line(points = {{150, 50}, {145, 50}, {145, 30}, {130, 30}}, color = {95, 95, 95}));
  connect(leftUCABody.frame_a, leftWishboneUprightLoop.upperFrame_o) annotation(
    Line(visible = not quasiStatic, points = {{-95, 100}, {-85, 100}, {-85, 79}, {-69, 79}}, color = {95, 95, 95}));
  connect(leftLCABody.frame_a, leftWishboneUprightLoop.lowerFrame_o) annotation(
    Line(visible = not quasiStatic, points = {{-120, 8}, {-69, 8}, {-69, 21}}, color = {95, 95, 95}));
  connect(rightUCABody.frame_a, rightWishboneUprightLoop.upperFrame_o) annotation(
    Line(visible = not quasiStatic, points = {{95, 100}, {85, 100}, {85, 79}, {69, 79}}, color = {95, 95, 95}));
  connect(rightLCABody.frame_a, rightWishboneUprightLoop.lowerFrame_o) annotation(
    Line(visible = not quasiStatic, points = {{120, 8}, {69, 8}, {69, 21}}, color = {95, 95, 95}));
  connect(leftUnsprungBody.frame_a, leftTire.unsprungFrame) annotation(
    Line(
      visible = not quasiStatic,
      points = {{-135, 85}, {-132, 85}, {-132, 75}, {-154, 75}, {-154, 60}},
      color = {95, 95, 95}));
  connect(rightUnsprungBody.frame_a, rightTire.unsprungFrame) annotation(
    Line(
      visible = not quasiStatic,
      points = {{135, 85}, {132, 85}, {132, 75}, {154, 75}, {154, 60}},
      color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-180, -180}, {180, 140}}, preserveAspectRatio = true)),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {
      Line(origin = {-160, 4}, points = {{-20, -4}, {40, -4}, {40, 6}}),
      Line(origin = {160, 4}, points = {{20, -4}, {-40, -4}, {-40, 6}}),
      Line(origin = {-130, 50}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {190, 50}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {0, 46}, points = {{0, 4}, {0, -12}})
    }),
    Documentation(info = "<html>
<p>
Partial model <code>PartialDoubleWishboneAxle</code> implements the shared double-wishbone geometry and bodies behind the architecture-neutral axle contract.
</p>
<p>
It provides chassis, wheel-hub, tire, steering, and suspension reference frames shared by direct, bellcrank, and stabilizer-bar axle variants.
</p>
<p>The default tires are MF52Tire with Wheel1DOF_Y. Replace either tire or its
wheel model on the axle; the vehicle chassis preserves those selections.
Equilibrium rigs explicitly replace the tires with BaseTire and stateless wheel
variants. Shared calibration and connector settings remain on the constraining
types so they survive replacement.</p>
</html>"));
end PartialDoubleWishboneAxle;
