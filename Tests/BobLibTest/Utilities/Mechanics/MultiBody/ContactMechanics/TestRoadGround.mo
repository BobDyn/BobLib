within BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics;
model TestRoadGround

  inner Modelica.Mechanics.MultiBody.World world(g = 0, enableAnimation = false)
    annotation(Placement(transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}})));

  inner replaceable VehicleInterfaces.Roads.FlatRoad road(animation = false)
    constrainedby VehicleInterfaces.Roads.Interfaces.Base
    annotation(Placement(transformation(origin = {20, 50}, extent = {{-20, -10}, {20, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed point(r = road.position(2, 3)-0.01*road.normal(2, 3))
    annotation(Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadGround dut(r = {2, 3, 0})
    annotation(Placement(transformation(origin = {20, -20}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(point.frame_b, dut.frame_b) annotation(Line(points = {{-60, 20}, {20, 20}, {20, 0}}, color = {95, 95, 95}));
  assert(abs(dut.f_z-1000.000252499937)<1e-7, "Reference penetration load");
  assert(sum((Modelica.Mechanics.MultiBody.Frames.resolve1(dut.frame_b.R, dut.frame_b.f)+dut.f_z*road.normal(2, 3)).^2)<1e-10,
    "Ground force follows the surface normal");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-8), Diagram(coordinateSystem(extent = {{-100, -80}, {80, 80}})));
end TestRoadGround;
