within BobLib.Vehicle.Chassis.Suspension;

model FrAxleDW_BC "Double wishbone front axle with bellcranks and stabar"
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDWRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  // Record parameters
  parameter AxleDWRecord pAxle;
  parameter StabarRecord pStabar;
  
  extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;
  // Left pushrod
  // Left bellcrank
  Linkages.Bellcrank3 leftBellcrank(pivot = pAxle.bellcrankPivot, pivotAxis = pAxle.bellcrankPivotAxis, pickup_1 = pAxle.bellcrankPickup1, pickup_2 = pAxle.bellcrankPickup2, pickup_3 = pAxle.bellcrankPickup3, linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  // Left shock
  Linkages.ShockLinkage leftShockLinkage(r_a = pAxle.bellcrankPickup3, r_b = pAxle.shockMount, s_0 = pAxle.springFreeLength, springTable = pAxle.springTable, damperTable = pAxle.damperTable, n_a = pAxle.bellcrankPivotAxis, n_b = Vectors.normalize(pAxle.bellcrankPivot - pAxle.bellcrankPickup3), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  // Right pushrod
  // Right bellcrank
  Linkages.Bellcrank3 rightBellcrank(pivot = mirrorXZ(pAxle.bellcrankPivot), pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis), pickup_1 = mirrorXZ(pAxle.bellcrankPickup1), pickup_2 = mirrorXZ(pAxle.bellcrankPickup2), pickup_3 = mirrorXZ(pAxle.bellcrankPickup3), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

// Right shock
  Linkages.ShockLinkage rightShockLinkage(r_a = mirrorXZ(pAxle.bellcrankPickup3), r_b = mirrorXZ(pAxle.shockMount), s_0 = pAxle.springFreeLength, springTable = pAxle.springTable, damperTable = pAxle.damperTable, n_a = mirrorXZ(pAxle.bellcrankPivotAxis), n_b = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivot - pAxle.bellcrankPickup3)), linkDiameter = linkDiameter, jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  // Stabar
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
equation
  connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
    Line(points = {{-40, -20}, {-30, -20}}, color = {95, 95, 95}));
  connect(leftBellcrank.pickupFrame3, leftShockLinkage.frame_a) annotation(
    Line(points = {{-50, -30}, {-50, -40}}, color = {95, 95, 95}));
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-50, -70}, {-30, -70}}, color = {95, 95, 95}));
  connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
    Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(rightBellcrank.pickupFrame3, rightShockLinkage.frame_a) annotation(
    Line(points = {{50, -30}, {50, -40}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{50, -70}, {30, -70}}, color = {95, 95, 95}));
  connect(axleFrame, toLeftBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {-10, -20}}));
  connect(axleFrame, toRightBellcrank.frame_a) annotation(
    Line(points = {{0, 0}, {0, -20}, {10, -20}}));
  connect(axleFrame, toLeftShock.frame_a) annotation(
    Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(points = {{0, 114}, {0, 140}}));
  connect(toRightShock.frame_a, axleFrame) annotation(
    Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})}));
end FrAxleDW_BC;