within BobLibTest.TestVehicle.TestAero;

model TestCFDAeroMap

  import BobLib.Records.VehicleRecord.Aero.CFDAeroMapRecord;

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
    mzTable = [9, 10; 11, 12]) "Aerodynamic force and moment map calibration at the specified reference conditions" annotation(Dialog(group = "Calibration"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner VehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere(
    v = {-20, 0, 0}, ambientPressure = pAero.referenceDensity*287.0512249529787*293.15)
    annotation(Placement(transformation(origin = {-40, -70}, extent = {{-20, -10}, {20, 10}})));

  BobLib.Aero.CFDAeroMap aero(pAero = pAero) annotation(
    Placement(transformation(origin = {80, 0}, extent = {{-25, -15.625}, {25, 15.625}}, rotation = 0)));

  BobLib.Utilities.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(origin = {20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  BobLib.Chassis.Interfaces.ChassisBus chassisBus annotation(
    Placement(transformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1}) annotation(
      Placement(transformation(origin = {-130, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedMount(animation = false) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant frontRideHeight(k = 0.03) annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Sources.Constant rearRideHeight(k = 0.03) annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

equation
  connect(controlBus, aero.controlBus) annotation(
    Line(points = {{20, 60}, {61.25, 60}, {61.25, 15.625}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus.chassisBus, chassisBus) annotation(
    Line(points = {{20, 60}, {-40, 60}}, color = {255, 204, 51}, thickness = 0.5));

  connect(fixedMount.frame_b, aero.sprungChassisFrame) annotation(
    Line(points = {{30, 0}, {55, 0}}, color = {95, 95, 95}));
  connect(frontRideHeight.y, chassisBus.rideHeight_1) annotation(
    Line(points = {{-89, 60}, {-40, 60}}, color = {0, 0, 127}));
  connect(frontRideHeight.y, chassisBus.rideHeight_2) annotation(
    Line(points = {{-89, 60}, {-80, 60}, {-80, 66}, {-40, 66}, {-40, 60}}, color = {0, 0, 127}));
  connect(rearRideHeight.y, chassisBus.rideHeight_3) annotation(
    Line(points = {{-89, 0}, {-60, 0}, {-60, 50}, {-40, 50}, {-40, 60}}, color = {0, 0, 127}));
  connect(rearRideHeight.y, chassisBus.rideHeight_4) annotation(
    Line(points = {{-89, 0}, {-70, 0}, {-70, 40}, {-40, 40}, {-40, 60}}, color = {0, 0, 127}));

  assert(abs(aero.drag - 52.0) < 1e-9, "CFDAeroMap drag interpolation changed");
  assert(abs(aero.downforce - 520.0) < 1e-9, "CFDAeroMap downforce interpolation changed");
  assert(abs(aero.force[1] + aero.drag) < 1e-9, "CFDAeroMap x-force sign changed");
  assert(abs(aero.force[3] + aero.downforce) < 1e-9, "CFDAeroMap z-force sign changed");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
  annotation(Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}}, preserveAspectRatio = true)));
end TestCFDAeroMap;
