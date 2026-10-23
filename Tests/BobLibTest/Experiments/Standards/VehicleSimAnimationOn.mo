within BobLibTest.Experiments.Standards;

model VehicleSimAnimationOn

  extends BobLib.Experiments.Standards.VehicleSim(headless = false);
  Modelica.Blocks.Routing.RealPassThrough measuredLoads[4] annotation(Placement(transformation(origin = {110, -120}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Routing.RealPassThrough measuredLongitudinalVelocity annotation(Placement(transformation(origin = {110, -150}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
  connect(measuredLongitudinalVelocity.u, chassis.controlBus.chassisBus.longitudinalVelocity) annotation(Line(points = {{98, -150}, {57, -150}, {57, -38}, {60.375, -38}}, color = {0, 0, 127}));
  assert(abs(measuredLongitudinalVelocity.y - chassis.bodyVelocity[1]) < 1e-8,
    "VehicleInterfaces longitudinalVelocity must publish signed chassis-forward velocity");
  connect(measuredLoads[1].u, chassis.controlBus.chassisBus.Fz_1) annotation(Line(points = {{98, -120}, {90, -120}, {90, -104}, {45, -104}, {45, -38}, {60.375, -38}}, color = {0, 0, 127}));
  connect(measuredLoads[2].u, chassis.controlBus.chassisBus.Fz_2) annotation(Line(points = {{98, -120}, {90, -120}, {90, -108}, {48, -108}, {48, -38}, {60.375, -38}}, color = {0, 0, 127}));
  connect(measuredLoads[3].u, chassis.controlBus.chassisBus.Fz_3) annotation(Line(points = {{98, -120}, {90, -120}, {90, -112}, {51, -112}, {51, -38}, {60.375, -38}}, color = {0, 0, 127}));
  connect(measuredLoads[4].u, chassis.controlBus.chassisBus.Fz_4) annotation(Line(points = {{98, -120}, {90, -120}, {90, -116}, {54, -116}, {54, -38}, {60.375, -38}}, color = {0, 0, 127}));
  assert(max(abs({measuredLoads[1].y, measuredLoads[2].y, measuredLoads[3].y, measuredLoads[4].y} -
    {chassis.Fz_1, chassis.Fz_2, chassis.Fz_3, chassis.Fz_4})) < 1e-8,
    "The standard chassis bus must publish the physical tire loads in front-left, front-right, rear-left, rear-right order");
  annotation(experiment(StartTime = 0, StopTime = 0.02, Tolerance = 1e-06, Interval = 0.002));
  annotation(Diagram(coordinateSystem(extent = {{-200, -180}, {180, 120}}, preserveAspectRatio = true)));
end VehicleSimAnimationOn;
