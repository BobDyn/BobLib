within BobLib.Vehicle.Chassis.Suspension;

model RrAxleDW_BC_ARB "Double wishbone rear axle with bellcranks and stabar"
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  // Record parameters
  parameter AxleDWRecord pAxle;
  parameter StabarRecord pStabar;
  extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase(rodToLower = false, rodInboard = pAxle.bellcrankPickup1);
  // Left pullrod
  // Left bellcrank
  Linkages.Bellcrank3 leftBellcrank(pivot = pAxle.bellcrankPivot, pivotAxis = pAxle.bellcrankPivotAxis, pickup_1 = pAxle.bellcrankPickup1, pickup_2 = pAxle.bellcrankPickup2, pickup_3 = pAxle.bellcrankPickup3, linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  // Left shock
  Linkages.ShockLinkage leftShockLinkage(r_a = pAxle.bellcrankPickup2, r_b = pAxle.shockMount, s_0 = pAxle.springFreeLength, springTable = pAxle.springTable, damperTable = pAxle.damperTable, n_a = pAxle.bellcrankPivotAxis, n_b = Vectors.normalize(pAxle.bellcrankPivot - pAxle.bellcrankPickup2), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-80, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  // Right pullrod
  // Right bellcrank
  Linkages.Bellcrank3 rightBellcrank(pivot = mirrorXZ(pAxle.bellcrankPivot), pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis), pickup_1 = mirrorXZ(pAxle.bellcrankPickup1), pickup_2 = mirrorXZ(pAxle.bellcrankPickup2), pickup_3 = mirrorXZ(pAxle.bellcrankPickup3), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  // Right shock
  Linkages.ShockLinkage rightShockLinkage(r_a = mirrorXZ(pAxle.bellcrankPickup2), r_b = mirrorXZ(pAxle.shockMount), s_0 = pAxle.springFreeLength, springTable = pAxle.springTable, damperTable = pAxle.damperTable, n_a = mirrorXZ(pAxle.bellcrankPivotAxis), n_b = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivot - pAxle.bellcrankPickup2)), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {80, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  // Stabar
  Templates.Stabar.Stabar stabar(pStabar = pStabar, jointDiameter = jointDiameter, linkDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {0, -116}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod rightDroplink(r_a = mirrorXZ(pStabar.leftArmEnd),
                                                               r_b = mirrorXZ(pAxle.bellcrankPickup3),
                                                               n1_a = {0, 1, 0},
                                                               kinematicConstraint = true,
                                                               linkDiameter = linkDiameter,
                                                               jointDiameter = jointDiameter,
                                                               show_universal_axes = false) annotation(
    Placement(transformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod leftDroplink(r_a = pStabar.leftArmEnd,
                                                              r_b = pAxle.bellcrankPickup3,
                                                              n1_a = {0, 1, 0},
                                                              kinematicConstraint = true,
                                                              linkDiameter = linkDiameter,
                                                              jointDiameter = jointDiameter,
                                                              show_universal_axes = false) annotation(
    Placement(transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
    Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));

protected
  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftBellcrank(r = pAxle.bellcrankPivot - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(r = pAxle.shockMount - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightBellcrank(r = mirrorXZ(pAxle.bellcrankPivot) - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(r = mirrorXZ(pAxle.shockMount) - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toStabar(r = {pStabar.leftBarEnd[1], 0, pStabar.leftBarEnd[3]} - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
    Line(points = {{-40, -20}, {-30, -20}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-80, -70}, {-30, -70}}, color = {95, 95, 95}));
  connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
    Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{80, -70}, {30, -70}}, color = {95, 95, 95}));
  connect(axleFrame, toLeftBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {-10, -20}}));
  connect(axleFrame, toRightBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {10, -20}}));
  connect(axleFrame, toLeftShock.frame_a) annotation(
    Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
  connect(toRightBellcrank.frame_a, toRightShock.frame_a) annotation(
    Line(points = {{10, -20}, {0, -20}, {0, -70}, {10, -70}}, color = {95, 95, 95}));
  connect(axleFrame, toStabar.frame_a) annotation(
    Line(points = {{0, 0}, {0, -80}}));
  connect(toStabar.frame_b, stabar.supportFrame) annotation(
    Line(points = {{0, -100}, {0, -110}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame2) annotation(
    Line(points = {{-80, -40}, {-80, -20}, {-60, -20}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame2) annotation(
    Line(points = {{80, -40}, {80, -20}, {60, -20}}, color = {95, 95, 95}));
  connect(leftBellcrank.pickupFrame3, leftDroplink.frame_b) annotation(
    Line(points = {{-50, -30}, {-50, -80}}, color = {95, 95, 95}));
  connect(leftDroplink.frame_a, stabar.leftArmFrame) annotation(
    Line(points = {{-50, -100}, {-50, -120}, {-20, -120}}, color = {95, 95, 95}));
  connect(rightBellcrank.pickupFrame3, rightDroplink.frame_b) annotation(
    Line(points = {{50, -30}, {50, -80}}, color = {95, 95, 95}));
  connect(rightDroplink.frame_a, stabar.rightArmFrame) annotation(
    Line(points = {{50, -100}, {50, -120}, {20, -120}}, color = {95, 95, 95}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(points = {{0, 114}, {0, 140}}));
  connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1) annotation(
    Line(points = {{-100, -30}, {-70, -30}, {-70, -10}, {-50, -10}}, color = {95, 95, 95}));
  connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1) annotation(
    Line(points = {{100, -30}, {70, -30}, {70, -10}, {50, -10}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 120}}, preserveAspectRatio = true)));
end RrAxleDW_BC_ARB;