within BobLibTest.Regression;

model TireForceFrameSmoke

  import SI = Modelica.Units.SI;

  inner parameter Real linkDiameter = 0.02;
  inner parameter Boolean headless = true;

  parameter
    BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord
    pWheel(
      R0 = 0.3,
      rimR0 = 0.2,
      rimWidth = 0.2,
      staticAlpha = 15,
      staticGamma = 10);

  model InclinedTire

    parameter
      BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord
      pTestWheel;

    extends BobLib.Chassis.Suspension.Tires.BaseTire(
      pPartialWheel = pTestWheel,
      realExpressionFy(y = 1000),
      realExpressionMz(y = 100));

    output SI.Force appliedForceWorld[3] = tireForceWorld;
    output SI.Torque appliedTorqueWorld[3] = tireTorqueWorld;
  end InclinedTire;

  output SI.Force appliedForceWorld[3] = tire.appliedForceWorld;
  output SI.Torque appliedTorqueWorld[3] = tire.appliedTorqueWorld;

  inner Modelica.Mechanics.MultiBody.World world(
    n = {0, 0, -1},
    enableAnimation = false);

  Modelica.Mechanics.MultiBody.Parts.Fixed chassis(
    r = {0, 0, 0.3});

  InclinedTire tire(pTestWheel = pWheel);

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
  connect(chassis.frame_b, tire.chassisFrame);

  annotation(
    experiment(StartTime = 0, StopTime = 0.001, Tolerance = 1e-6, Interval = 0.001));
end TireForceFrameSmoke;
