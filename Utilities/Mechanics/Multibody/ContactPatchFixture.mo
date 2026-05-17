within BobLib.Utilities.Mechanics.Multibody;

model ContactPatchFixture
  import Modelica.SIunits;
  
  parameter SIunits.Position CP_init[3] "Vector from origin to initial contact patch location, resolved in world frame";
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {70, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 20}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  
  Modelica.Mechanics.MultiBody.Parts.Fixed CP_Fixed(r = CP_init, animation = false) annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_x(animation = false, n = {1, 0, 0}) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_y(animation = false, n = {0, 1, 0}) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical DOF_xyz(animation = false) annotation(
    Placement(transformation(origin = {70, 50}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // Hack for preventing singular poses
  Modelica.Mechanics.MultiBody.Joints.Revolute Revolute(animation = false, n = {1, 0, 0}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Components.Disc AngleOffset(deltaPhi = Modelica.Constants.pi/2) annotation(
    Placement(transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(DOF_x.frame_a, CP_Fixed.frame_b) annotation(
    Line(points = {{-60, 0}, {-70, 0}, {-70, -40}}, color = {95, 95, 95}));
  connect(DOF_y.frame_a, DOF_x.frame_b) annotation(
    Line(points = {{-10, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(DOF_y.frame_b, Revolute.frame_a) annotation(
    Line(points = {{10, 0}, {40, 0}}, color = {95, 95, 95}));
  connect(Revolute.frame_b, DOF_xyz.frame_a) annotation(
    Line(points = {{60, 0}, {70, 0}, {70, 40}}, color = {95, 95, 95}));
  connect(AngleOffset.flange_b, Revolute.axis) annotation(
    Line(points = {{40, 30}, {50, 30}, {50, 10}}));
  connect(Revolute.support, AngleOffset.flange_a) annotation(
    Line(points = {{44, 10}, {10, 10}, {10, 30}, {20, 30}}));
  connect(frame_a, DOF_xyz.frame_b) annotation(
    Line(points = {{70, 100}, {70, 60}}));
annotation(
    Icon(graphics = {Line(origin = {0, -20}, points = {{-20, 0}, {20, 0}}, thickness = 1), Line(origin = {-18, -23}, points = {{-2, -3}, {2, 3}}, thickness = 1), Line(origin = {-10, -23}, points = {{-2, -3}, {2, 3}}, thickness = 1), Line(origin = {-2, -23}, points = {{-2, -3}, {2, 3}}, thickness = 1), Line(origin = {6, -23}, points = {{-2, -3}, {2, 3}}, thickness = 1), Line(origin = {14, -23}, points = {{-2, -3}, {2, 3}}, thickness = 1), Line(points = {{0, -20}, {0, 20}})}));
end ContactPatchFixture;
