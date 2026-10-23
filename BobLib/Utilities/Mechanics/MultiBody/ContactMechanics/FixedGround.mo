within BobLib.Utilities.Mechanics.MultiBody.ContactMechanics;

model FixedGround "Vertical contact with a fixed ground reference"

  extends BobLib.Icons.GroundPhysicsIcon;

  parameter Modelica.Units.SI.Position r[3] = zeros(3) "Ground reference position in world coordinates" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.TranslationalSpringConstant c(min = 0) = 100000 "Vertical contact stiffness" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.TranslationalDampingConstant d(min = 0) = 750 "Vertical contact damping" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Length eps(min = Modelica.Constants.small) = 1e-6 "Smoothing length" annotation(Dialog(group = "Advanced"));
  parameter Modelica.Units.SI.Force forceEps(min = Modelica.Constants.small) = 1.0 "Smoothing force for normal-force clamp" annotation(Dialog(group = "Advanced"));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));

protected
  Modelica.Mechanics.MultiBody.Parts.Fixed support(r = r, animation = false) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}})));

  GroundPhysics contact(c = c, d = d, eps = eps, forceEps = forceEps) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(support.frame_b, contact.frame_a) annotation(
    Line(points = {{-50, 40}, {0, 40}, {0, 10}}, color = {95, 95, 95}));
  connect(contact.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));

  annotation(Diagram(coordinateSystem(extent = {{-100, -40}, {100, 60}}, preserveAspectRatio = true)), Icon(graphics = {
    Line(points = {{-60, -45}, {60, -45}}, thickness = 1),
    Line(points = {{-60, -45}, {-40, -75}}, thickness = 0.5),
    Line(points = {{-30, -45}, {-10, -75}}, thickness = 0.5),
    Line(points = {{0, -45}, {20, -75}}, thickness = 0.5),
    Line(points = {{30, -45}, {50, -75}}, thickness = 0.5)
  }), Documentation(info = "<html><p>Combines GroundPhysics with a fixed world reference at r. The support symbol on the ground icon denotes the internal fixed reference. Use GroundPhysics directly when an external reference frame is required.</p></html>"));
end FixedGround;
