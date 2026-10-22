within BobLib.DriverEnvironments.Internal;
model ManeuverMeasurements "Chassis measurements consumed by the standard VI driver"

  extends Modelica.Blocks.Icons.Block;
  BobLib.Chassis.Interfaces.ChassisBus chassisBus annotation(Placement(transformation(origin = {-100, 0}, extent = {{-15, -15}, {15, 15}})));
  VehicleInterfaces.Interfaces.DriverInterface driverInterface annotation(Placement(transformation(origin = {100, 0}, extent = {{-15, -15}, {15, 15}})));
  Modelica.Blocks.Routing.RealPassThrough Fz_1 annotation(Placement(transformation(origin = {0, 80}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Routing.RealPassThrough Fz_2 annotation(Placement(transformation(origin = {0, 40}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Routing.RealPassThrough Fz_3 annotation(Placement(transformation(origin = {0, 0}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Routing.RealPassThrough Fz_4 annotation(Placement(transformation(origin = {0, -40}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Routing.RealPassThrough bodyAcceleration_2 annotation(Placement(transformation(origin = {0, -80}, extent = {{-8, -8}, {8, 8}})));

equation
  connect(chassisBus.Fz_1, Fz_1.u) annotation(Line(points = {{-100, 0}, {-70, 0}, {-70, 80}, {-9.6, 80}}, color = {0, 0, 127}));
  connect(Fz_1.y, driverInterface.Fz_1) annotation(Line(points = {{8.8, 80}, {70, 80}, {70, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(chassisBus.Fz_2, Fz_2.u) annotation(Line(points = {{-100, 0}, {-65, 0}, {-65, 40}, {-9.6, 40}}, color = {0, 0, 127}));
  connect(Fz_2.y, driverInterface.Fz_2) annotation(Line(points = {{8.8, 40}, {65, 40}, {65, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(chassisBus.Fz_3, Fz_3.u) annotation(Line(points = {{-100, 0}, {-70, 0}, {-70, 0}, {-9.6, 0}}, color = {0, 0, 127}));
  connect(Fz_3.y, driverInterface.Fz_3) annotation(Line(points = {{8.8, 0}, {70, 0}, {70, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(chassisBus.Fz_4, Fz_4.u) annotation(Line(points = {{-100, 0}, {-75, 0}, {-75, -40}, {-9.6, -40}}, color = {0, 0, 127}));
  connect(Fz_4.y, driverInterface.Fz_4) annotation(Line(points = {{8.8, -40}, {75, -40}, {75, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(chassisBus.bodyAcceleration_2, bodyAcceleration_2.u) annotation(Line(points = {{-100, 0}, {-80, 0}, {-80, -80}, {-9.6, -80}}, color = {0, 0, 127}));
  connect(bodyAcceleration_2.y, driverInterface.bodyAcceleration_2) annotation(Line(points = {{8.8, -80}, {80, -80}, {80, 0}, {100, 0}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
end ManeuverMeasurements;
