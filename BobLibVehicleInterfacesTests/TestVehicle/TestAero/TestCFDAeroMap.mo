within BobLibVehicleInterfacesTests.TestVehicle.TestAero;

model TestCFDAeroMap
  import BobLibVehicleInterfaces.Records.VehicleRecord.Aero.CFDAeroMapRecord;

  inner parameter Boolean headless = false;

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

  BobLibVehicleInterfaces.Aero.CFDAeroMap aero(pAero = pAero) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1}) annotation(
      Placement(transformation(origin = {-80, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedMount(animation = false) annotation(
    Placement(transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant frontRideHeight(k = 0.03) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant rearRideHeight(k = 0.03) annotation(
    Placement(transformation(origin = {-60, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant relativeAirSpeed(k = 20) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant airDensity(k = pAero.referenceDensity) annotation(
    Placement(transformation(origin = {-60, -20}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(fixedMount.frame_b, aero.sprungChassisFrame) annotation(
    Line(points = {{-30, -40}, {-20, -40}, {-20, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(frontRideHeight.y, aero.frontRideHeight) annotation(
    Line(points = {{-49, 40}, {0, 40}, {0, 12}}, color = {0, 0, 127}));
  connect(rearRideHeight.y, aero.rearRideHeight) annotation(
    Line(points = {{-49, 20}, {0, 20}, {0, 12}}, color = {0, 0, 127}));
  connect(relativeAirSpeed.y, aero.relativeAirSpeed) annotation(
    Line(points = {{-49, 0}, {0, 0}, {0, 12}}, color = {0, 0, 127}));
  connect(airDensity.y, aero.airDensity) annotation(
    Line(points = {{-49, -20}, {0, -20}, {0, 12}}, color = {0, 0, 127}));

  assert(abs(aero.drag - 52.0) < 1e-9, "CFDAeroMap drag interpolation changed");
  assert(abs(aero.downforce - 520.0) < 1e-9, "CFDAeroMap downforce interpolation changed");
  assert(abs(aero.force[1] + aero.drag) < 1e-9, "CFDAeroMap x-force sign changed");
  assert(abs(aero.force[3] + aero.downforce) < 1e-9, "CFDAeroMap z-force sign changed");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestCFDAeroMap;
