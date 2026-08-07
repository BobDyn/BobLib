within BobLibTest.TestVehicle.TestAero;

model TestCFDAeroMapDirections

  import SI = Modelica.Units.SI;
  import BobLib.Records.VehicleRecord.Aero.CFDAeroMapRecord;

  model AeroScenario

    parameter CFDAeroMapRecord pAero;
    parameter SI.Velocity bodySpeed;
    parameter SI.Velocity windVelocityWorld[3];

    output SI.Force force[3] = aero.force;
    output SI.Force drag = aero.drag;
    output SI.Force downforce = aero.downforce;

    BobLib.Aero.CFDAeroMap aero(
      pAero = pAero,
      headless = true);

  protected

    VehicleInterfaces.Interfaces.ControlBus controlBus;
    VehicleInterfaces.Interfaces.ChassisBus chassisBus;
    BobLib.Atmospheres.Interfaces.AtmosphereBus atmosphereBus;

    Modelica.Mechanics.MultiBody.Parts.Fixed motionReference(
      animation = false);

    Modelica.Mechanics.MultiBody.Joints.Prismatic vehicleMotion(
      n = {1, 0, 0},
      useAxisFlange = true,
      animation = false);

    Modelica.Mechanics.Translational.Sources.Position prescribedPosition(
      exact = true,
      useSupport = true);

    Modelica.Blocks.Sources.RealExpression bodyPosition(
      y = bodySpeed*time);

    Modelica.Blocks.Sources.Constant rideHeight(
      k = 0.03);

    Modelica.Blocks.Sources.RealExpression windVelocity[3](
      y = windVelocityWorld);

    Modelica.Blocks.Sources.Constant airDensity(
      k = pAero.referenceDensity);

    Modelica.Blocks.Sources.Constant airTemperature(
      k = 293.15);

    Modelica.Blocks.Sources.Constant relativeHumidity(
      k = 0);

    Modelica.Blocks.Sources.Constant pressure(
      k = 101325);

  equation
    connect(controlBus, aero.controlBus);
    connect(controlBus.chassisBus, chassisBus);
    connect(atmosphereBus, aero.atmosphereBus);

    connect(motionReference.frame_b, vehicleMotion.frame_a);
    connect(vehicleMotion.frame_b, aero.sprungChassisFrame);
    connect(bodyPosition.y, prescribedPosition.s_ref);
    connect(prescribedPosition.flange, vehicleMotion.axis);
    connect(prescribedPosition.support, vehicleMotion.support);

    connect(rideHeight.y, chassisBus.rideHeight_1);
    connect(rideHeight.y, chassisBus.rideHeight_2);
    connect(rideHeight.y, chassisBus.rideHeight_3);
    connect(rideHeight.y, chassisBus.rideHeight_4);
    connect(windVelocity.y, atmosphereBus.windVelocityWorld);
    connect(airDensity.y, atmosphereBus.airDensity);
    connect(airTemperature.y, atmosphereBus.airTemperature);
    connect(relativeHumidity.y, atmosphereBus.relativeHumidity);
    connect(pressure.y, atmosphereBus.pressure);
  end AeroScenario;

  parameter CFDAeroMapRecord pAero(
    referenceSpeed = 10,
    referenceDensity = 1.225,
    aeroRef = {0, 0, 0},
    FL_RideHeightRef = {1, 0.5, 0},
    RL_RideHeightRef = {-1, 0.5, 0},
    frontRideHeightGrid = {0.02, 0.04},
    rearRideHeightGrid = {0.02, 0.04},
    dragTable = [10, 12; 14, 16],
    downforceTable = [100, 120; 140, 160],
    mxTable = [1, 2; 3, 4],
    myTable = [5, 6; 7, 8],
    mzTable = [9, 10; 11, 12]);

  AeroScenario reverseVehicle(
    pAero = pAero,
    bodySpeed = -10,
    windVelocityWorld = {0, 0, 0});

  AeroScenario followingWind(
    pAero = pAero,
    bodySpeed = 10,
    windVelocityWorld = {20, 0, 0});

  AeroScenario crosswind(
    pAero = pAero,
    bodySpeed = 10,
    windVelocityWorld = {0, 10, 0});

protected

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity);

equation
  assert(
    abs(reverseVehicle.force[1] - reverseVehicle.drag) < 1e-9,
    "Drag must oppose reverse vehicle motion");
  assert(
    abs(followingWind.force[1] - followingWind.drag) < 1e-9,
    "Drag must point forward when a following wind exceeds vehicle speed");
  assert(
    abs(crosswind.force[1] + crosswind.drag/sqrt(2)) < 1e-9,
    "Crosswind drag x component must oppose body-relative airflow");
  assert(
    abs(crosswind.force[2] - crosswind.drag/sqrt(2)) < 1e-9,
    "Crosswind drag y component must oppose body-relative airflow");
  assert(
    abs(crosswind.force[3] + crosswind.downforce) < 1e-9,
    "CFD downforce must remain resolved along negative body z");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestCFDAeroMapDirections;
