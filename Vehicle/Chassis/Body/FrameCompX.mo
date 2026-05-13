within BobLib.Vehicle.Chassis.Body;

model FrameCompX
  extends BobLib.Vehicle.Chassis.Body.FrameBase;
  import Modelica.SIunits;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.MassRecord;
  
  parameter SIunits.RotationalSpringConstant torsionalStiff;
  parameter MassRecord pSprungMass;
  parameter SIunits.RotationalDampingConstant torsionalDamp = 2 * sqrt(torsionalStiff * pSprungMass.inertia[1, 1]);
  
  Modelica.Mechanics.MultiBody.Joints.Revolute torsionalRevolute(n = {1, 0, 0}, useAxisFlange = true, phi(nominal=1e-4), w(start = 0, fixed = true)) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Modelica.Mechanics.Rotational.Components.Spring spring(c = torsionalStiff, phi_rel0 = 0) annotation(
    Placement(transformation(origin = {20, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = torsionalDamp)  annotation(
    Placement(transformation(origin = {20, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

equation
  connect(midToAft.frame_a, torsionalRevolute.frame_a) annotation(
    Line(points = {{40, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(midToFore.frame_a, torsionalRevolute.frame_b) annotation(
    Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(spring.flange_a, torsionalRevolute.support) annotation(
    Line(points = {{30, -30}, {40, -30}, {40, -10}, {6, -10}}));
  connect(spring.flange_b, torsionalRevolute.axis) annotation(
    Line(points = {{10, -30}, {0, -30}, {0, -10}}));
  connect(damper.flange_a, torsionalRevolute.support) annotation(
    Line(points = {{30, -50}, {40, -50}, {40, -10}, {6, -10}}));
  connect(damper.flange_b, torsionalRevolute.axis) annotation(
    Line(points = {{10, -50}, {0, -50}, {0, -10}}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Line(origin = {0.03, -1}, rotation = -90, points = {{-1.02758, -20}, {-1.02758, -6}, {-5.02758, -4}, {2.97242, 0}, {-5.02758, 4}, {2.97242, 8}, {-1.02758, 10}, {-1.02758, 20}}, color = {0, 170, 0}, thickness = 2), Text(textColor = {255, 0, 0}, extent = {{-14, 14}, {14, -14}}, textString = "rxDOF")}));
end FrameCompX;
