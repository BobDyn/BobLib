within BobLib.Vehicle.Chassis.Suspension;

model FrAxleDW_Direct "Double wishbone axle with direct-acting suspension"
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  import BobLib.Utilities.Math.Vector.mirrorXZ;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_DirectRecord;
  
  // Record parameters
  parameter AxleDW_DirectRecord pAxle;
  
  extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;
  
  // Left shock
  BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(r_a = pLeftDW.rodMount,
                                                                           r_b = pAxle.shockMount,
                                                                           s_0 = pAxle.springFreeLength,
                                                                           springTable = pAxle.springTable,
                                                                           damperTable = pAxle.damperTable,
                                                                           n_a = {1, 0, 0},
                                                                           n_b = {0, 1, 0},
                                                                           linkDiameter = linkDiameter,
                                                                           jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-50, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  
  // Right shock
  BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(r_a = mirrorXZ(pLeftDW.rodMount),
                                                                            r_b = mirrorXZ(pAxle.shockMount),
                                                                            s_0 = pAxle.springFreeLength,
                                                                            springTable = pAxle.springTable,
                                                                            damperTable = pAxle.damperTable,
                                                                            n_a = {1, 0, 0},
                                                                            n_b = mirrorXZ({0, 1, 0}),
                                                                            linkDiameter = linkDiameter,
                                                                            jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {50, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));
  
  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
    Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));
  
protected
  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(r = pAxle.shockMount - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(r = mirrorXZ(pAxle.shockMount) - effectiveCenter, animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
  
equation
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-50, -60}, {-50, -70}, {-30, -70}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{50, -60}, {50, -70}, {30, -70}}, color = {95, 95, 95}));
  connect(axleFrame, toLeftShock.frame_a) annotation(
    Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
  connect(rackAndPinion.pinionFlange, steerFlange) annotation(
    Line(points = {{0, 114}, {0, 140}}));
  connect(toRightShock.frame_a, axleFrame) annotation(
    Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftShockLinkage.frame_a) annotation(
    Line(points = {{-100, -10}, {-110, -10}, {-110, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightShockLinkage.frame_a) annotation(
    Line(points = {{100, -10}, {110, -10}, {110, -30}, {50, -30}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})}));
end FrAxleDW_Direct;