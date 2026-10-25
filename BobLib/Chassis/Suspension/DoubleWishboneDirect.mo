within BobLib.Chassis.Suspension;

model DoubleWishboneDirect "Double wishbone axle with direct-acting suspension"

  import Modelica.Units.SI;
  import BobLib.Utilities.Math.Vector.mirrorXZ;

  // Record parameters

  extends BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle(redeclare record AxleRecord = BobLib.Records.VehicleRecord.Chassis.Suspension.DoubleWishboneDirectRecord);

  replaceable model SpringModel = BobLib.Chassis.Suspension.Components.TabularSpring(
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable, s_0 = pAxle.actuation.springFreeLength)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Spring law used on both axle sides" annotation(choicesAllMatching = true);
  replaceable model DamperModel = BobLib.Chassis.Suspension.Components.TabularDamper(
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable)
    constrainedby Modelica.Mechanics.Translational.Interfaces.PartialTwoFlanges
    "Damper law used on both axle sides" annotation(choicesAllMatching = true);

  // Left shock
  BobLib.Chassis.Suspension.Components.ShockLinkage leftShockLinkage(
    useDamping = not quasiStatic,
    redeclare replaceable SpringModel TabularSpring,
    redeclare replaceable DamperModel TabularDamper,
    s_0 = pAxle.actuation.springFreeLength,
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable,
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable) annotation(
    Placement(transformation(origin = {-60, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

  // Right shock
  BobLib.Chassis.Suspension.Components.ShockLinkage rightShockLinkage(
    useDamping = not quasiStatic,
    redeclare replaceable SpringModel TabularSpring,
    redeclare replaceable DamperModel TabularDamper,
    s_0 = pAxle.actuation.springFreeLength,
    springPoints = pAxle.actuation.springPoints, springTable = pAxle.actuation.springTable,
    damperPoints = pAxle.actuation.damperPoints, damperTable = pAxle.actuation.damperTable) annotation(
    Placement(transformation(origin = {60, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

protected
  final parameter SI.Position leftRodWishboneMount[3] = if pAxle.actuation.rodToLower then pAxle.geometry.lower_o
    else pAxle.geometry.upper_o
    "Outboard wishbone joint used as the spring/damper apex reference";

  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
    r = pAxle.actuation.shockMount - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {-20, -85}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
    r = mirrorXZ(pAxle.actuation.shockMount) - effectiveCenter,
    animation = false) annotation(
    Placement(transformation(origin = {20, -85}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
    r = pAxle.actuation.rodMount - leftRodWishboneMount,
    animation = false) annotation(
    Placement(transformation(origin = {-90, -20}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
    r = mirrorXZ(pAxle.actuation.rodMount - leftRodWishboneMount),
    animation = false) annotation(
    Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = 0;

  if pAxle.actuation.rodToLower then
    connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o) annotation(
      Line(
        visible = pAxle.actuation.rodToLower,
        points = {{-80, -20}, {-69, -20}, {-69, 21}},
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
        points = {{-80, -20}, {-80, 17}, {-35, 17}, {-35, 90}, {-69, 90}, {-69, 79}},
        color = {95, 95, 95}));
    connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o) annotation(
      Line(
        visible = not pAxle.actuation.rodToLower,
        points = {{80, -10}, {80, 17}, {35, 17}, {35, 90}, {69, 90}, {69, 79}},
        color = {95, 95, 95}));
  end if;
  connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
    Line(points = {{-60, -60}, {-60, -85}, {-30, -85}}, color = {95, 95, 95}));
  connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
    Line(points = {{60, -60}, {60, -85}, {30, -85}}, color = {95, 95, 95}));
  connect(toLeftLower_i.frame_a, toLeftShock.frame_a) annotation(
    Line(points = {{-10, 30}, {0, 30}, {0, -85}, {-10, -85}}));
  connect(toRightShock.frame_a, toLeftShock.frame_a) annotation(
    Line(points = {{10, -85}, {-10, -85}}, color = {95, 95, 95}));
  connect(toLeftApex.frame_b, leftShockLinkage.frame_a) annotation(
    Line(points = {{-100, -20}, {-103, -20}, {-103, -34}, {-68, -34}, {-68, -24}, {-60, -24}, {-60, -30}}, color = {95, 95, 95}));
  connect(toRightApex.frame_b, rightShockLinkage.frame_a) annotation(
    Line(points = {{100, -10}, {103, -10}, {103, -24}, {60, -24}, {60, -30}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -180}, {180, 140}}, preserveAspectRatio = true)),
    Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Model <code>DoubleWishboneDirect</code> implements a double-wishbone axle with direct spring/damper actuation.
</p>
<p>
It mirrors left-side geometry to the right side and connects wishbone/upright loops, steering rack, tires, mass properties, and spring/damper or stabilizer-bar linkages for the detailed chassis.
</p>
</html>"));

end DoubleWishboneDirect;
