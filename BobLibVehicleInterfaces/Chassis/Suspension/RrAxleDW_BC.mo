within BobLibVehicleInterfaces.Chassis.Suspension;

model RrAxleDW_BC "Double wishbone axle with bellcranks mounting to shock and push/pullrod"
  import SI = Modelica.Units.SI;
  import Modelica.Math.Vectors;
  import BobLibVehicleInterfaces.Utilities.Math.Vector.mirrorXZ;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.AxleDW_BCRecord;

  // Record parameters
  parameter AxleDW_BCRecord pAxle;

  extends BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;
  extends BobLibVehicleInterfaces.Icons.SteeringWheelOverlayIcon;

  // Left bellcrank
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.Bellcrank2 leftBellcrank(
    pivot = pAxle.bellcrankPivot,
    pivotAxis = pAxle.bellcrankPivotAxis,
    pickup_1 = pAxle.bellcrankRodPickup,
    pickup_2 = pAxle.bellcrankShockPickup,
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  // Left shock
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(
    r_a = pAxle.bellcrankShockPickup,
    r_b = pAxle.shockMount,
    s_0 = pAxle.springFreeLength,
    springTable = pAxle.springTable,
    damperTable = pAxle.damperTable,
    n_a = pAxle.bellcrankPivotAxis,
    n_b = Vectors.normalize(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup),
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

  // Right bellcrank
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.Bellcrank2 rightBellcrank(
    pivot = mirrorXZ(pAxle.bellcrankPivot),
    pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis),
    pickup_1 = mirrorXZ(pAxle.bellcrankRodPickup),
    pickup_2 = mirrorXZ(pAxle.bellcrankShockPickup),
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  // Right shock
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(
    r_a = mirrorXZ(pAxle.bellcrankShockPickup),
    r_b = mirrorXZ(pAxle.shockMount),
    s_0 = pAxle.springFreeLength,
    springTable = pAxle.springTable,
    damperTable = pAxle.damperTable,
    n_a = mirrorXZ(pAxle.bellcrankPivotAxis),
    n_b = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup)),
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
    Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

protected
  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftBellcrank(
    r = pAxle.bellcrankPivot - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
    r = pAxle.shockMount - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightBellcrank(
    r = mirrorXZ(pAxle.bellcrankPivot) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
    r = mirrorXZ(pAxle.shockMount) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
    r = pAxle.rodMount - pLeftDW.upper_o,
    animation = false) annotation(
    Placement(transformation(origin = {-80, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
    r = mirrorXZ(pAxle.rodMount - pLeftDW.upper_o),
    animation = false) annotation(
    Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));

public
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.Rod leftPushrod(
    jointDiameter = jointDiameter,
    kinematicConstraint = true,
    linkDiameter = linkDiameter,
    n1_a = Vectors.normalize(pAxle.bellcrankPivotAxis),
    r_a = pAxle.bellcrankRodPickup,
    r_b = pAxle.rodMount) annotation(
    Placement(transformation(origin = {-120, -30}, extent = {{20, -20}, {-20, 20}})));
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.Rod rightPushrod(
    jointDiameter = jointDiameter,
    kinematicConstraint = true,
    linkDiameter = linkDiameter,
    n1_a = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivotAxis)),
    r_a = mirrorXZ(pAxle.bellcrankRodPickup),
    r_b = mirrorXZ(pAxle.rodMount)) annotation(
    Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(toLeftApex.frame_a, leftWishboneUprightLoop.upperFrame_o) annotation(
    Line(points = {{-70, -20}, {-68, -20}, {-68, 78}}, color = {95, 95, 95}));
  connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o) annotation(
    Line(points = {{80, -10}, {70, -10}, {70, 78}}, color = {95, 95, 95}));
  connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1) annotation(
    Line(points = {{-100, -30}, {-70, -30}, {-70, -10}, {-50, -10}}, color = {95, 95, 95}));
  connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1) annotation(
    Line(points = {{100, -30}, {70, -30}, {70, -10}, {50, -10}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame2) annotation(
    Line(points = {{-50, -40}, {-60, -40}, {-60, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame2) annotation(
    Line(points = {{50, -40}, {60, -40}, {60, -20}}, color = {95, 95, 95}));

  connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
    Line(points = {{-40, -20}, {-30, -20}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-50, -70}, {-30, -70}}, color = {95, 95, 95}));
  connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
    Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{50, -70}, {30, -70}}, color = {95, 95, 95}));
  connect(axleFrame, toLeftBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {-10, -20}}));
  connect(axleFrame, toRightBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {10, -20}}));
  connect(axleFrame, toLeftShock.frame_a) annotation(
    Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
  connect(toRightBellcrank.frame_a, toRightShock.frame_a) annotation(
    Line(points = {{10, -20}, {0, -20}, {0, -70}, {10, -70}}, color = {95, 95, 95}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(points = {{0, 114}, {0, 140}}));
  connect(toRightShock.frame_a, axleFrame) annotation(
    Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
    Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
    Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
    Icon(graphics = {
      Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5)
    }),
    Documentation(info = "<html>
<p>
Model <code>RrAxleDW_BC</code> implements a rear double-wishbone axle with bellcrank-actuated spring/damper motion.
</p>
<p>
It mirrors left-side geometry to the right side and connects wishbone/upright loops, steering rack, tires, mass properties, and spring/damper or stabilizer-bar linkages for the detailed chassis.
</p>
</html>"));

end RrAxleDW_BC;
