within BobLibTest.Chassis.Suspension.Components;

model TestStabar

  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;

  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));

  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Real jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-170, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed mountFixture(
    r = {pVehicle.chassis.frontAxle.stabilizer.leftBarEnd[1], 0, pVehicle.chassis.frontAxle.stabilizer.leftBarEnd[3]},
    animation = false) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  BobLib.Chassis.Suspension.Components.Stabar stabar(
    pStabar = pVehicle.chassis.frontAxle.stabilizer,
    linkDiameter = linkDiameter,
    jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {0, -4}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, sphereDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(mountFixture.frame_b, stabar.supportFrame) annotation(
    Line(points = {{0, -30}, {0, -10}}, color = {95, 95, 95}));
  connect(body.frame_a, stabar.rightArmFrame) annotation(
    Line(points = {{30, 0}, {20, 0}}, color = {95, 95, 95}));
  assert(abs(stabar.leftArmFrame.r_0[1] - stabar.rightArmFrame.r_0[1]) < 1e-7 and
    abs(stabar.leftArmFrame.r_0[3] - stabar.rightArmFrame.r_0[3]) < 1e-7,
    "The unloaded massless arm must follow the loaded arm without bar twist");
  assert(abs(stabar.leftArmFrame.r_0[2] - stabar.rightArmFrame.r_0[2] -
    2*pVehicle.chassis.frontAxle.stabilizer.leftArmEnd[2]) < 1e-7,
    "Stabilizer rotation must preserve the lateral pickup spacing");
  annotation(
    experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.002),
    Diagram(coordinateSystem(extent = {{-180, -80}, {80, 60}}, preserveAspectRatio = true)),
    Icon(coordinateSystem(extent = {{-140, -60}, {140, 60}})));
end TestStabar;
