within BobLibVehicleInterfaces.Chassis.Suspension;

model FrAxleDW_Direct "Double wishbone axle with direct-acting suspension"
  import SI = Modelica.Units.SI;
  import Modelica.Math.Vectors;
  import BobLibVehicleInterfaces.Utilities.Math.Vector.mirrorXZ;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.AxleDW_DirectRecord;

  // Record parameters
  parameter AxleDW_DirectRecord pAxle;

  extends BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;

  // Left shock
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(
    r_a = pAxle.rodMount,
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
  BobLibVehicleInterfaces.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(
    r_a = mirrorXZ(pAxle.rodMount),
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
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
    r = pAxle.shockMount - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
    r = mirrorXZ(pAxle.shockMount) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
        r = pAxle.rodMount - pLeftDW.lower_o,
        animation = false) annotation(
        Placement(transformation(origin = {-80, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
        r = mirrorXZ(pAxle.rodMount - pLeftDW.lower_o),
        animation = false) annotation(
        Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o) annotation(
    Line(points = {{-70, -20}, {-68, -20}, {-68, 22}}, color = {95, 95, 95}));
  connect(toRightApex.frame_a, rightWishboneUprightLoop.lowerFrame_o) annotation(
    Line(points = {{80, -10}, {70, -10}, {70, 22}}, color = {95, 95, 95}));
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
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 120}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Model <code>FrAxleDW_Direct</code> implements a front double-wishbone axle with direct spring/damper actuation.
</p>
<p>
It mirrors left-side geometry to the right side and connects wishbone/upright loops, steering rack, tires, mass properties, and spring/damper or stabilizer-bar linkages for the detailed chassis.
</p>
</html>"));

end FrAxleDW_Direct;
