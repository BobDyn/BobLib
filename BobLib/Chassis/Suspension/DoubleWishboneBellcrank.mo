within BobLib.Chassis.Suspension;

model DoubleWishboneBellcrank "Double wishbone axle with bellcranks mounting to shock and push/pullrod"

  import Modelica.Units.SI;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;

  // Record parameters

  extends BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle(redeclare record AxleRecord = BobLib.Records.VehicleRecord.Chassis.Suspension.DoubleWishboneBellcrankRecord);

  replaceable model SpringModel = BobLib.Chassis.Suspension.Components.TabularSpring(
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable, s_0 = pAxle.actuation.springFreeLength)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Spring law used on both axle sides" annotation(choicesAllMatching = true);
  replaceable model DamperModel = BobLib.Chassis.Suspension.Components.TabularDamper(
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Damper law used on both axle sides" annotation(choicesAllMatching = true);

  // Left bellcrank
  BobLib.Chassis.Suspension.Components.Bellcrank2 leftBellcrank(
    pivot = pAxle.actuation.bellcrankPivot,
    pivotAxis = pAxle.actuation.bellcrankPivotAxis,
    pickup_1 = pAxle.actuation.bellcrankRodPickup,
    pickup_2 = pAxle.actuation.bellcrankShockPickup,
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  // Left shock
  BobLib.Chassis.Suspension.Components.ShockLinkage leftShockLinkage(
    useDamping = not quasiStatic,
    redeclare replaceable SpringModel TabularSpring,
    redeclare replaceable DamperModel TabularDamper,
    s_0 = pAxle.actuation.springFreeLength,
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable,
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable) annotation(
    Placement(transformation(origin = {-60, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

  // Right bellcrank
  BobLib.Chassis.Suspension.Components.Bellcrank2 rightBellcrank(
    pivot = mirrorXZ(pAxle.actuation.bellcrankPivot),
    pivotAxis = mirrorXZ(pAxle.actuation.bellcrankPivotAxis),
    pickup_1 = mirrorXZ(pAxle.actuation.bellcrankRodPickup),
    pickup_2 = mirrorXZ(pAxle.actuation.bellcrankShockPickup),
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  // Right shock
  BobLib.Chassis.Suspension.Components.ShockLinkage rightShockLinkage(
    useDamping = not quasiStatic,
    redeclare replaceable SpringModel TabularSpring,
    redeclare replaceable DamperModel TabularDamper,
    s_0 = pAxle.actuation.springFreeLength,
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable,
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable) annotation(
    Placement(transformation(origin = {60, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

protected
  final parameter SI.Position leftRodWishboneMount[3] = if pAxle.actuation.rodToLower then pAxle.geometry.lower_o
    else pAxle.geometry.upper_o
    "Outboard wishbone joint used as the push/pullrod apex reference";

  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftBellcrank(
    r = pAxle.actuation.bellcrankPivot - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-25, -20}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
    r = pAxle.actuation.shockMount - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -85}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightBellcrank(
    r = mirrorXZ(pAxle.actuation.bellcrankPivot) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {25, -20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
    r = mirrorXZ(pAxle.actuation.shockMount) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, -85}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
    r = pAxle.actuation.rodMount - leftRodWishboneMount,
    animation = false) annotation(
    Placement(transformation(origin = {-90, -10}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
    r = mirrorXZ(pAxle.actuation.rodMount - leftRodWishboneMount),
    animation = false) annotation(
    Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));

public
  BobLib.Chassis.Suspension.Components.Rod leftPushrod(
    jointDiameter = jointDiameter,
    kinematicConstraint = true,
    linkDiameter = linkDiameter,
    n1_a = Vectors.normalize(pAxle.actuation.bellcrankPivotAxis),
    r_a = pAxle.actuation.bellcrankRodPickup,
    r_b = pAxle.actuation.rodMount) annotation(
    Placement(transformation(origin = {-120, -30}, extent = {{20, -20}, {-20, 20}})));

  BobLib.Chassis.Suspension.Components.Rod rightPushrod(
    jointDiameter = jointDiameter,
    kinematicConstraint = true,
    linkDiameter = linkDiameter,
    n1_a = Vectors.normalize(mirrorXZ(pAxle.actuation.bellcrankPivotAxis)),
    r_a = mirrorXZ(pAxle.actuation.bellcrankRodPickup),
    r_b = mirrorXZ(pAxle.actuation.rodMount)) annotation(
    Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}})));

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = 0;

  if pAxle.actuation.rodToLower then
    connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o) annotation(
      Line(
        visible = pAxle.actuation.rodToLower,
        points = {{-80, -10}, {-69, -10}, {-69, 21}},
        color = {95, 95, 95}));
    connect(toRightApex.frame_a, rightWishboneUprightLoop.lowerFrame_o) annotation(
      Line(
        visible = pAxle.actuation.rodToLower,
        points = {{80, -10}, {69, -10}, {69, 21}},
        color = {95, 95, 95}));
  else
    connect(toLeftApex.frame_a, leftWishboneUprightLoop.upperFrame_o) annotation(
      Line(
        visible = not pAxle.actuation.rodToLower,
        points = {{-80, -10}, {-80, 17}, {-35, 17}, {-35, 90}, {-69, 90}, {-69, 79}},
        color = {95, 95, 95}));
    connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o) annotation(
      Line(
        visible = not pAxle.actuation.rodToLower,
        points = {{80, -10}, {80, 17}, {35, 17}, {35, 90}, {69, 90}, {69, 79}},
        color = {95, 95, 95}));
  end if;
  connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1) annotation(
    Line(points = {{-100, -30}, {-75, -30}, {-75, -5}, {-50, -5}, {-50, -10}}, color = {95, 95, 95}));
  connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1) annotation(
    Line(points = {{100, -30}, {75, -30}, {75, -5}, {50, -5}, {50, -10}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame2) annotation(
    Line(points = {{-60, -40}, {-66, -40}, {-66, -20}, {-60, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame2) annotation(
    Line(points = {{60, -40}, {66, -40}, {66, -20}, {60, -20}}, color = {95, 95, 95}));

  connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
    Line(points = {{-40, -20}, {-35, -20}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-60, -70}, {-60, -85}, {-30, -85}}, color = {95, 95, 95}));
  connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
    Line(points = {{35, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{60, -70}, {60, -85}, {30, -85}}, color = {95, 95, 95}));
  connect(toLeftLower_i.frame_a, toLeftBellcrank.frame_a) annotation(
    Line(points = {{-10, 30}, {0, 30}, {0, -20}, {-15, -20}}));
  connect(toLeftBellcrank.frame_a, toRightBellcrank.frame_a) annotation(
    Line(points = {{-15, -20}, {15, -20}}));
  connect(toLeftBellcrank.frame_a, toLeftShock.frame_a) annotation(
    Line(points = {{-15, -20}, {0, -20}, {0, -85}, {-10, -85}}));
  connect(toLeftShock.frame_a, toRightShock.frame_a) annotation(
    Line(points = {{-10, -85}, {10, -85}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
    Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
    Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -180}, {180, 140}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Model <code>DoubleWishboneBellcrank</code> implements a double-wishbone axle with bellcrank-actuated spring/damper motion.
</p>
<p>
It mirrors left-side geometry to the right side and connects wishbone/upright loops, steering rack, tires, mass properties, and spring/damper or stabilizer-bar linkages for the detailed chassis.
</p>
</html>"));

end DoubleWishboneBellcrank;
