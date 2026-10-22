within BobLib.Chassis.Suspension;

model DoubleWishboneBellcrankStabar "Double wishbone axle with bellcranks mounting to shock, push/pullrod, and stabar"

  import Modelica.Units.SI;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;

  // Record parameters

  final parameter SI.Position bellcrankVisualPickups[3, 3] = {
    if pAxle.actuation.rodPickup == i then pAxle.actuation.bellcrankRodPickup
      elseif pAxle.actuation.shockPickup == i then pAxle.actuation.bellcrankShockPickup
      else pAxle.actuation.bellcrankStabarPickup for i in 1:3} "Bellcrank pickup coordinates arranged by the selected rod, shock and stabilizer indices";

  extends BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle(redeclare record AxleRecord = BobLib.Records.VehicleRecord.Chassis.Suspension.DoubleWishboneBellcrankStabarRecord);

  replaceable model SpringModel = BobLib.Chassis.Suspension.Components.TabularSpring(
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable, s_0 = pAxle.actuation.springFreeLength)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Spring law used on both axle sides" annotation(choicesAllMatching = true);
  replaceable model DamperModel = BobLib.Chassis.Suspension.Components.TabularDamper(
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Damper law used on both axle sides" annotation(choicesAllMatching = true);

  // Left bellcrank
  BobLib.Chassis.Suspension.Components.Bellcrank3 leftBellcrank(
    pivot = pAxle.actuation.bellcrankPivot,
    pivotAxis = pAxle.actuation.bellcrankPivotAxis,
    pickup_1 = pAxle.actuation.bellcrankRodPickup,
    pickup_2 = pAxle.actuation.bellcrankShockPickup,
    pickup_3 = pAxle.actuation.bellcrankStabarPickup,
    visualPickups = bellcrankVisualPickups,
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
  BobLib.Chassis.Suspension.Components.Bellcrank3 rightBellcrank(
    pivot = mirrorXZ(pAxle.actuation.bellcrankPivot),
    pivotAxis = mirrorXZ(pAxle.actuation.bellcrankPivotAxis),
    pickup_1 = mirrorXZ(pAxle.actuation.bellcrankRodPickup),
    pickup_2 = mirrorXZ(pAxle.actuation.bellcrankShockPickup),
    pickup_3 = mirrorXZ(pAxle.actuation.bellcrankStabarPickup),
    visualPickups = {mirrorXZ(bellcrankVisualPickups[i, :]) for i in 1:3},
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

  // Stabar
  BobLib.Chassis.Suspension.Components.Stabar stabar(
    pStabar = pAxle.stabilizer,
    jointDiameter = jointDiameter,
    linkDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {0, -145}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));

  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical rightDroplink(
    rodLength = Vectors.norm(mirrorXZ(pAxle.actuation.bellcrankStabarPickup - pAxle.stabilizer.leftArmEnd)),
    sphereDiameter = jointDiameter,
    rodDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {85, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical leftDroplink(
    rodLength = Vectors.norm(pAxle.actuation.bellcrankStabarPickup - pAxle.stabilizer.leftArmEnd),
    sphereDiameter = jointDiameter,
    rodDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {-85, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

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

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toStabar(
    r = {pAxle.stabilizer.leftBarEnd[1], 0, pAxle.stabilizer.leftBarEnd[3]} - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

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
  stabarAngle = stabar.spring.phi_rel;

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
  connect(leftDroplink.frame_b, leftBellcrank.pickupFrame3) annotation(
    Line(points = {{-85, -90}, {-85, -35}, {-70, -35}, {-70, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(rightDroplink.frame_b, rightBellcrank.pickupFrame3) annotation(
    Line(points = {{85, -90}, {85, -35}, {70, -35}, {70, -30}, {50, -30}}, color = {95, 95, 95}));

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
  connect(toRightShock.frame_a, toStabar.frame_a) annotation(
    Line(points = {{10, -85}, {0, -85}, {0, -100}}));
  connect(toStabar.frame_b, stabar.supportFrame) annotation(
    Line(points = {{0, -120}, {0, -139}}, color = {95, 95, 95}));
  connect(stabar.rightArmFrame, rightDroplink.frame_a) annotation(
    Line(points = {{20, -149}, {85, -149}, {85, -110}}, color = {95, 95, 95}));
  connect(stabar.leftArmFrame, leftDroplink.frame_a) annotation(
    Line(points = {{-20, -149}, {-85, -149}, {-85, -110}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
    Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
    Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -180}, {180, 140}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Model <code>DoubleWishboneBellcrankStabar</code> implements a double-wishbone axle with bellcrank-actuated spring/damper motion and a stabilizer bar.
</p>
<p>
It mirrors left-side geometry to the right side and connects wishbone/upright loops, steering rack, tires, mass properties, and spring/damper or stabilizer-bar linkages for the detailed chassis.
</p>
</html>"));
end DoubleWishboneBellcrankStabar;
