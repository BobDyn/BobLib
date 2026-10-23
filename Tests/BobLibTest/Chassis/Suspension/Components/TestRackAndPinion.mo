within BobLibTest.Chassis.Suspension.Components;

model TestRackAndPinion

  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;

  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));

  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-190, -110}, extent = {{-10, -10}, {10, 10}})));

  // Rack definition
  Modelica.Mechanics.MultiBody.Parts.Fixed rackFixed(
    r = {pVehicle.chassis.frontAxle.steering.leftPickup[1], 0, pVehicle.chassis.frontAxle.steering.leftPickup[3]},
    animation = false) annotation(
    Placement(transformation(origin = {0, -65}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  BobLib.Chassis.Suspension.Components.RackAndPinion rackAndPinion(
    pRack = pVehicle.chassis.frontAxle.steering,
    linkDiameter = linkDiameter) annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));

  // Pinion input
  Modelica.Blocks.Sources.Ramp pinionRamp(
    height = 100*Modelica.Constants.pi/180,
    duration = 1,
    startTime = 1) annotation(
    Placement(transformation(origin = {-70, 65}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Position pinionPosition(exact = true) annotation(
    Placement(transformation(origin = {-10, 65}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  // Torque feedback
  Modelica.Mechanics.MultiBody.Parts.Fixed springSupport(
    r = {pVehicle.chassis.frontAxle.steering.leftPickup[1], pVehicle.chassis.frontAxle.steering.leftPickup[2] + 0.5, pVehicle.chassis.frontAxle.steering.leftPickup[3]},
    animation = false) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Forces.Spring spring(c = 1e6, s_unstretched = 0.5, width = linkDiameter*2) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(rackFixed.frame_b, rackAndPinion.mountFrame) annotation(
    Line(points = {{0, -55}, {0, -4}}, color = {95, 95, 95}));
  connect(pinionPosition.flange, rackAndPinion.pinionFlange) annotation(
    Line(points = {{0, 65}, {0, 4}}, color = {0, 0, 0}));
  connect(pinionRamp.y, pinionPosition.phi_ref) annotation(
    Line(points = {{-59, 65}, {-22, 65}}, color = {0, 0, 127}));
  connect(springSupport.frame_b, spring.frame_a) annotation(
    Line(points = {{-70, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(spring.frame_b, rackAndPinion.leftFrame) annotation(
    Line(points = {{-40, 0}, {-20, 0}}, color = {95, 95, 95}));
  assert(abs((rackAndPinion.leftFrame.r_0[2] + rackAndPinion.rightFrame.r_0[2])/2 -
    pinionRamp.y*pVehicle.chassis.frontAxle.steering.cFactor/(2*Modelica.Constants.pi)) < 1e-8,
    "Rack midpoint travel must match the commanded pinion rotation and conversion factor");
  assert(abs(rackAndPinion.leftFrame.r_0[2] - rackAndPinion.rightFrame.r_0[2] -
    2*pVehicle.chassis.frontAxle.steering.leftPickup[2]) < 1e-8,
    "Rack motion must preserve left/right pickup separation");
  assert(abs(rackAndPinion.mountFrame.t[1] + rackAndPinion.pinionFlange.tau) < 1e-7,
    "Pinion torque must react into the rack housing about its steering shaft axis");
  annotation(experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.002));
  annotation(Diagram(coordinateSystem(extent = {{-200, -120}, {120, 100}}, preserveAspectRatio = true)));
end TestRackAndPinion;
