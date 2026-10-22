within BobLibTest.Chassis.Suspension.Tires;

model TireForceFrameSmoke

  import Modelica.Units.SI;

  inner VehicleInterfaces.Roads.FlatRoad road annotation(Placement(transformation(origin = {30, 50}, extent = {{-20, -10}, {20, 10}})));

  parameter
    BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord
    pWheel(
      R0 = 0.3,
      rimR0 = 0.2,
      rimWidth = 0.2,
      staticAlpha = 15,
      staticGamma = 10) "Wheel radius, rim dimensions and static alignment used by the fixture" annotation(Dialog(group = "Calibration"));
  inner parameter Real linkDiameter = 0.02 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = true "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  model InclinedTire

    parameter
      BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord
      pTestWheel "Wheel geometry passed into the contact-frame probe" annotation(Dialog(group = "Calibration"));

    parameter BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));

    extends BobLib.Chassis.Suspension.Tires.BaseTire(
      pTire(wheel(geometry = pTestWheel), force = pVehicle.chassis.frontAxle.tire.force),
      realExpressionFy(y = 1000),
      realExpressionMz(y = 100));

    output SI.Force appliedForceWorld[3] = tireForceWorld;
    output SI.Torque appliedTorqueWorld[3] = tireTorqueWorld;

  initial equation
    assert(
      abs(e_xg*e_yg) < 1e-12,
      "Ground-projected tire axes must be orthogonal");
    assert(
      abs(sqrt((600*e_xg + 800*e_yg)*(600*e_xg + 800*e_yg)) - 1000) < 1e-8,
      "Combined road-plane force magnitude must be basis invariant");
  end InclinedTire;

  output SI.Force appliedForceWorld[3] = tire.appliedForceWorld;
  output SI.Torque appliedTorqueWorld[3] = tire.appliedTorqueWorld;

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = false) annotation(Placement(transformation(origin = {-130, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Fixed chassis(
    r = {0, 0, 0.3}) annotation(Placement(transformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  InclinedTire tire(pTestWheel = pWheel) annotation(Placement(transformation(origin = {30, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));

initial equation
  assert(
    abs(appliedForceWorld[3]) < 1e-8,
    "Tire shear force must remain in the road plane");
  assert(
    abs(sqrt(appliedForceWorld[1]^2 + appliedForceWorld[2]^2) - 1000) < 1e-8,
    "Road-plane force magnitude must be preserved");
  assert(
    abs(appliedTorqueWorld[3] - 100) < 1e-8,
    "Aligning moment must act about the road normal");

equation
  connect(chassis.frame_b, tire.chassisFrame) annotation(Line(points = {{-90, -20}, {10, -20}}, color = {95, 95, 95}));

  annotation(
    experiment(StartTime = 0, StopTime = 0.001, Tolerance = 1e-6, Interval = 0.001));
  annotation(Diagram(coordinateSystem(extent = {{-140, -100}, {80, 100}}, preserveAspectRatio = true)));
end TireForceFrameSmoke;
