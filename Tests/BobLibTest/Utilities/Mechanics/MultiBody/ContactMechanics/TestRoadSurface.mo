within BobLibTest.Utilities.Mechanics.MultiBody.ContactMechanics;
model TestRoadSurface

model CurvedRoad

  extends VehicleInterfaces.Roads.FlatRoad(animation = false,
    redeclare function position = curvePosition,
    redeclare function normal = curveNormal,
    redeclare function headingDirection = curveHeading);
  function curvePosition

    extends VehicleInterfaces.Roads.Interfaces.positionBase;

  algorithm
    r_0:={s, w, 0.05*s*s};
  end curvePosition;
  function curveNormal

    extends VehicleInterfaces.Roads.Interfaces.normalBase;

  algorithm
    e_n_0:={-0.1*s, 0, 1}/sqrt(1+0.01*s*s);
  end curveNormal;
  function curveHeading

    extends VehicleInterfaces.Roads.Interfaces.headingDirectionBase;

  algorithm
    e_s_0:={1, 0, 0.1*s}/sqrt(1+0.01*s*s);
  end curveHeading;
end CurvedRoad;

  inner Modelica.Mechanics.MultiBody.World world(g = 0, enableAnimation = false)
    annotation(Placement(transformation(origin = {-90, -70}, extent = {{-10, -10}, {10, 10}})));
  inner CurvedRoad road annotation(Placement(transformation(origin = {-60, 50}, extent = {{-20, -10}, {20, 10}})));
  Real coordinate = 2+time;
  Modelica.Blocks.Sources.RealExpression point[3](y = {coordinate, 3, 0.05*coordinate^2}-0.01*{-0.1*coordinate, 0, 1}/sqrt(1+0.01*coordinate^2))
    annotation(Placement(transformation(origin = {-54.5, 0}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.RoadSurface dut(s_start = 2, w_start = 3)
    annotation(Placement(transformation(origin = {40, 0}, extent = {{-25, -15}, {25, 15}})));

equation
  connect(point.y, dut.r) annotation(Line(points = {{-43.5, 0}, {15, 0}}, color = {0, 0, 127}));
  assert(abs(dut.s-coordinate)<1e-8 and abs(dut.w-3)<1e-8, "Curved surface projection");
  assert(abs(dut.gap+0.01)<1e-8, "Curved normal gap");
  annotation(experiment(StopTime = 0.5, Tolerance = 1e-8), Diagram(coordinateSystem(extent = {{-100, -80}, {100, 80}})));
end TestRoadSurface;
