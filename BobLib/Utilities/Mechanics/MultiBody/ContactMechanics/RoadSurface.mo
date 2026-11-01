within BobLib.Utilities.Mechanics.MultiBody.ContactMechanics;
model RoadSurface "Local orthogonal projection onto a VI road surface"

  parameter Boolean flatRoadFastPath = false "Specialize horizontal z=0 geometry; assert the VI road agrees" annotation(Dialog(group = "Advanced"), choices(checkBox = true), Evaluate = true);
  outer VehicleInterfaces.Roads.Interfaces.Base road;

  Modelica.Blocks.Interfaces.RealInput r[3](each unit = "m") "Query point in world coordinates" annotation(Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  parameter Modelica.Units.SI.Position s_start = 0 "Initial longitudinal surface coordinate" annotation(Dialog(group = "Initialization"));
  parameter Modelica.Units.SI.Position w_start = 0 "Initial lateral surface coordinate" annotation(Dialog(group = "Initialization"));
  Modelica.Units.SI.Position s(start = s_start);
  Modelica.Units.SI.Position w(start = w_start);
  output Modelica.Units.SI.Position position[3];
  output Modelica.Units.SI.Position gap "Signed normal distance, positive above road";
  Modelica.Blocks.Interfaces.RealOutput normal[3] annotation(Placement(transformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}})));
  output Real heading[3];
  Modelica.Blocks.Interfaces.RealOutput mu "Road surface friction coefficient" annotation(Placement(transformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}})));

protected
  Real lateral[3];
  Modelica.Units.SI.Position delta[3];

equation
  if flatRoadFastPath then
    s = r[1];
    w = r[2];
    position = {s, w, 0};
    normal = {0, 0, 1};
    heading = {1, 0, 0};
  else
    position = road.position(s, w);
    normal = road.normal(s, w);
    heading = road.headingDirection(s, w);
    0 = delta*heading;
    0 = delta*lateral;
  end if;
  lateral = cross(normal, heading);
  delta = r-position;
  assert(not flatRoadFastPath or
    (sum((road.position(s, w)-{s, w, 0}).^2)<1e-12 and
     sum((road.normal(s, w)-{0, 0, 1}).^2)<1e-12 and
     sum((road.headingDirection(s, w)-{1, 0, 0}).^2)<1e-12),
    "flatRoadFastPath requires horizontal z=0 road geometry; disable it for other VI roads");
  gap = delta*normal;
  mu = road.frictionCoefficient(s, w);
  assert(abs(normal*normal-1)<1e-6 and abs(heading*heading-1)<1e-6 and abs(normal*heading)<1e-6,
    "VI road must return orthogonal unit normal and heading vectors");
  assert(mu>=0, "Road friction must be nonnegative");
  annotation(Icon(coordinateSystem(extent = {{-100, -60}, {100, 60}}), graphics = {
    Line(points = {{-80, -25}, {0, 0}, {80, 0}}, color = {95, 95, 95}, thickness = 1),
    Line(points = {{0, 0}, {0, 40}}, color = {0, 85, 170}, arrow = {Arrow.None, Arrow.Filled}),
    Text(extent = {{-100, 85}, {100, 65}}, textString = "%name", textColor = {0, 0, 255})}),
    Documentation(info = "<html><p>Projects a point onto the VI surface along its local normal. flatRoadFastPath explicitly reduces horizontal z=0 roads to s=r[1], w=r[2], gap=r[3], avoiding nonlinear projection systems when compiler calibration-preservation flags prevent function simplification. A runtime assertion checks that the road geometry agrees. Curved surfaces require a locally unique projection and appropriate coordinate start values. This is point-contact geometry, not a circular-wheel tangent construction.</p></html>"));
end RoadSurface;
