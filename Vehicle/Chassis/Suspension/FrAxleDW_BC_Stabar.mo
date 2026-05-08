within BobLib.Vehicle.Chassis.Suspension;

model FrAxleDW_BC_Stabar "Double wishbone axle with bellcranks mounting to shock, push/pullrod, and stabar"
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BC_StabarRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  
  // Record parameters
  parameter AxleDW_BC_StabarRecord pAxle;
  parameter StabarRecord pStabar;
  
  extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;
  
  // Left bellcrank
  BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank3 leftBellcrank(pivot = pAxle.bellcrankPivot,
                                                                      pivotAxis = pAxle.bellcrankPivotAxis,
                                                                      rRod = pAxle.bellcrankRodPickup,
                                                                      rShock = pAxle.bellcrankShockPickup,
                                                                      rStabar = pAxle.bellcrankStabarPickup,
                                                                      linkDiameter = linkDiameter,
                                                                      jointDiameter = jointDiameter,
                                                                      rodPickup = pAxle.rodPickup,
                                                                      shockPickup = pAxle.shockPickup,
                                                                      stabarPickup = pAxle.stabarPickup) annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  
  // Left shock
  BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(r_a = pAxle.bellcrankShockPickup,
                                                                           r_b = pAxle.shockMount,
                                                                           s_0 = pAxle.springFreeLength,
                                                                           springTable = pAxle.springTable,
                                                                           damperTable = pAxle.damperTable,
                                                                           n_a = pAxle.bellcrankPivotAxis,
                                                                           n_b = {0, 1, 0},
                                                                           linkDiameter = linkDiameter,
                                                                           jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  
  // Right bellcrank
  BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank3 rightBellcrank(pivot = mirrorXZ(pAxle.bellcrankPivot),
                                                                       pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis),
                                                                       rRod = mirrorXZ(pAxle.bellcrankRodPickup),
                                                                       rShock = mirrorXZ(pAxle.bellcrankShockPickup),
                                                                       rStabar = mirrorXZ(pAxle.bellcrankStabarPickup),
                                                                       linkDiameter = linkDiameter,
                                                                       jointDiameter = jointDiameter,
                                                                       rodPickup = pAxle.rodPickup,
                                                                       shockPickup = pAxle.shockPickup,
                                                                       stabarPickup = pAxle.stabarPickup) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  // Right shock
  BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(r_a = mirrorXZ(pAxle.bellcrankShockPickup),
                                                                            r_b = mirrorXZ(pAxle.shockMount),
                                                                            s_0 = pAxle.springFreeLength,
                                                                            springTable = pAxle.springTable,
                                                                            damperTable = pAxle.damperTable,
                                                                            n_a = mirrorXZ(pAxle.bellcrankPivotAxis),
                                                                            n_b = mirrorXZ({0, 1, 0}),
                                                                            linkDiameter = linkDiameter,
                                                                            jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  // Stabar
  BobLib.Vehicle.Chassis.Suspension.Templates.Stabar.Stabar stabar(pStabar = pStabar, jointDiameter = jointDiameter, linkDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {0, -116}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod rightDroplink(r_a = mirrorXZ(pStabar.leftArmEnd),
                                                               r_b = mirrorXZ(pAxle.bellcrankStabarPickup),
                                                               n1_a = {0, 1, 0},
                                                               kinematicConstraint = true,
                                                               linkDiameter = linkDiameter,
                                                               jointDiameter = jointDiameter,
                                                               show_universal_axes = false) annotation(
    Placement(transformation(origin = {70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod leftDroplink(r_a = pStabar.leftArmEnd,
                                                              r_b = pAxle.bellcrankStabarPickup,
                                                              n1_a = {0, 1, 0},
                                                              kinematicConstraint = true,
                                                              linkDiameter = linkDiameter,
                                                              jointDiameter = jointDiameter,
                                                              show_universal_axes = false) annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
    Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));
  
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
public
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod leftPushrod(jointDiameter = jointDiameter, kinematicConstraint = true, linkDiameter = linkDiameter, n1_a = {1, 0, 0}, r_a = pAxle.bellcrankRodPickup, r_b = pLeftDW.rodMount) annotation(
    Placement(transformation(origin = {-120, -30}, extent = {{20, -20}, {-20, 20}})));
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod rightPushrod(jointDiameter = jointDiameter, kinematicConstraint = true, linkDiameter = linkDiameter, n1_a = {1, 0, 0}, r_a = mirrorXZ(pAxle.bellcrankRodPickup), r_b = mirrorXZ(pLeftDW.rodMount)) annotation(
    Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}})));
equation
  if pAxle.rodPickup == 1 then
    connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1);
    connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1);
  elseif pAxle.shockPickup == 1 then
    connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame1);
    connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame1);
  elseif pAxle.stabarPickup == 1 then
    connect(leftDroplink.frame_b, leftBellcrank.pickupFrame1);
    connect(rightDroplink.frame_b, rightBellcrank.pickupFrame1);
  end if;
  
  if pAxle.rodPickup == 2 then
    connect(leftPushrod.frame_a, leftBellcrank.pickupFrame2);
    connect(rightPushrod.frame_a, rightBellcrank.pickupFrame2);
  elseif pAxle.shockPickup == 2 then
    connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame2);
    connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame2);
  elseif pAxle.stabarPickup == 2 then
    connect(leftDroplink.frame_b, leftBellcrank.pickupFrame2);
    connect(rightDroplink.frame_b, rightBellcrank.pickupFrame2);
  end if;
  
  if pAxle.rodPickup == 3 then
    connect(leftPushrod.frame_a, leftBellcrank.pickupFrame3);
    connect(rightPushrod.frame_a, rightBellcrank.pickupFrame3);
  elseif pAxle.shockPickup == 3 then
    connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame3);
    connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame3);
  elseif pAxle.stabarPickup == 3 then
    connect(leftDroplink.frame_b, leftBellcrank.pickupFrame3);
    connect(rightDroplink.frame_b, rightBellcrank.pickupFrame3);
  end if;
  
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
  connect(axleFrame, toStabar.frame_a) annotation(
    Line(points = {{0, 0}, {0, -80}}));
  connect(toStabar.frame_b, stabar.supportFrame) annotation(
    Line(points = {{0, -100}, {0, -110}}, color = {95, 95, 95}));
  connect(stabar.rightArmFrame, rightDroplink.frame_a) annotation(
    Line(points = {{20, -120}, {70, -120}, {70, -100}}, color = {95, 95, 95}));
  connect(stabar.leftArmFrame, leftDroplink.frame_a) annotation(
    Line(points = {{-20, -120}, {-70, -120}, {-70, -100}}, color = {95, 95, 95}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(points = {{0, 114}, {0, 140}}));
  connect(toRightShock.frame_a, axleFrame) annotation(
    Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
    Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
    Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})}));
end FrAxleDW_BC_Stabar;