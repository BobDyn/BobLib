within BobLibTest.Aero;

model TestCFDAeroMapDirections

  import Modelica.Units.SI;
  import BobLib.Records.VehicleRecord.Aero.CFDAeroMapRecord;

  model AeroScenario

    parameter CFDAeroMapRecord pAero "Aerodynamic force and moment map calibration at the specified reference conditions" annotation(Dialog(group = "Calibration"));
    parameter SI.Velocity bodySpeed "Prescribed longitudinal vehicle speed in the aero regression case" annotation(Dialog(group = "Operating settings"));
    parameter SI.Velocity windVelocityWorld[3] "Ambient wind velocity resolved in world coordinates" annotation(Dialog(group = "Operating settings"));

    output SI.Force force[3] = aero.force;
    output SI.Force drag = aero.drag;
    output SI.Force downforce = aero.downforce;

  inner VehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere(
    v = windVelocityWorld, ambientPressure = pAero.referenceDensity*287.0512249529787*293.15)
    annotation(Placement(transformation(origin = {-180, -60}, extent = {{-20, -10}, {20, 10}})));

    BobLib.Aero.CFDAeroMap aero(
      pAero = pAero,
      headless = true) annotation(Placement(transformation(origin = {100, 0}, extent = {{-25, -15.625}, {25, 15.625}}, rotation = 0)));

  protected

    BobLib.Utilities.Interfaces.ControlBus controlBus annotation(Placement(transformation(origin = {10, 70}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    BobLib.Chassis.Interfaces.ChassisBus chassisBus annotation(Placement(transformation(origin = {-80, 70}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));

    Modelica.Mechanics.MultiBody.Parts.Fixed motionReference(
      animation = false) annotation(Placement(transformation(origin = {20, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Modelica.Mechanics.MultiBody.Joints.Prismatic vehicleMotion(
      n = {1, 0, 0},
      useAxisFlange = true,
      animation = false) annotation(Placement(transformation(origin = {100, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Modelica.Mechanics.Translational.Sources.Position prescribedPosition(
      exact = true,
      useSupport = true) annotation(Placement(transformation(origin = {180, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

    Modelica.Blocks.Sources.RealExpression bodyPosition(
      y = bodySpeed*time) annotation(Placement(transformation(origin = {280, 160}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

    Modelica.Blocks.Sources.Constant rideHeight(
      k = 0.03) annotation(Placement(transformation(origin = {-200, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  equation
    connect(controlBus, aero.controlBus) annotation(Line(points = {{10, 70}, {10, 31}, {81.25, 31}, {81.25, 15.625}}, color = {255, 204, 51}));
    connect(controlBus.chassisBus, chassisBus) annotation(Line(points = {{10, 70}, {-80, 70}}, color = {255, 204, 51}));

    connect(motionReference.frame_b, vehicleMotion.frame_a) annotation(Line(points = {{30, 160}, {90, 160}}, color = {95, 95, 95}));
    connect(vehicleMotion.frame_b, aero.sprungChassisFrame) annotation(Line(points = {{110, 160}, {113, 160}, {113, 36}, {72, 36}, {72, 0}, {75, 0}}, color = {95, 95, 95}));
    connect(bodyPosition.y, prescribedPosition.s_ref) annotation(Line(points = {{269, 160}, {267, 160}, {267, 146}, {167, 146}, {167, 160}, {168, 160}}, color = {0, 0, 127}));
    connect(prescribedPosition.flange, vehicleMotion.axis) annotation(Line(points = {{190, 160}, {193, 160}, {193, 176}, {113, 176}, {113, 166}, {108, 166}}, color = {0, 0, 0}));
    connect(prescribedPosition.support, vehicleMotion.support) annotation(Line(points = {{180, 150}, {180, 136}, {87, 136}, {87, 176}, {96, 176}, {96, 166}}, color = {0, 0, 0}));

    connect(rideHeight.y, chassisBus.rideHeight_1) annotation(Line(points = {{-189, 70}, {-80, 70}}, color = {0, 0, 127}));
    connect(rideHeight.y, chassisBus.rideHeight_2) annotation(Line(points = {{-189, 70}, {-187, 70}, {-187, 66}, {-80, 66}, {-80, 70}}, color = {0, 0, 127}));
    connect(rideHeight.y, chassisBus.rideHeight_3) annotation(Line(points = {{-189, 70}, {-187, 70}, {-187, 76}, {-80, 76}, {-80, 70}}, color = {0, 0, 127}));
    connect(rideHeight.y, chassisBus.rideHeight_4) annotation(Line(points = {{-189, 70}, {-189, 61}, {-80, 61}, {-80, 70}}, color = {0, 0, 127}));
    annotation(Diagram(coordinateSystem(extent = {{-320, -220}, {340, 220}})), Icon(graphics = {Rectangle(extent = {{-100, 60}, {100, -60}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-140, 100}, {140, 70}}, textString = "%name", textColor = {0, 0, 255})}));
  end AeroScenario;

  parameter CFDAeroMapRecord pAero(
    FL_RideHeightRef = {1, 0.5, 0},
    RL_RideHeightRef = {-1, 0.5, 0},
    referenceSpeed = 10,
    referenceDensity = 1.225,
    aeroRef = {0, 0, 0},
    frontRideHeightGrid = {0.02, 0.04},
    rearRideHeightGrid = {0.02, 0.04},
    dragTable = [10, 12; 14, 16],
    downforceTable = [100, 120; 140, 160],
    mxTable = [1, 2; 3, 4],
    myTable = [5, 6; 7, 8],
    mzTable = [9, 10; 11, 12]) "Aerodynamic force and moment map calibration at the specified reference conditions";

  AeroScenario reverseVehicle(
    pAero = pAero,
    bodySpeed = -10,
    windVelocityWorld = {0, 0, 0}) annotation(Placement(transformation(origin = {-100, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));

  AeroScenario followingWind(
    pAero = pAero,
    bodySpeed = 10,
    windVelocityWorld = {20, 0, 0}) annotation(Placement(transformation(origin = {0, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));

  AeroScenario crosswind(
    pAero = pAero,
    bodySpeed = 10,
    windVelocityWorld = {0, 10, 0}) annotation(Placement(transformation(origin = {100, 0}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));

protected

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(Placement(transformation(origin = {-190, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

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
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {200, 120}})));
end TestCFDAeroMapDirections;
