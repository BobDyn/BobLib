within BobLib.Tests.TestVehicle.TestAero;

model TestCFDAeroMap
  import BobLib.Resources.VehicleRecord.Aero.CFDAeroMapRecord;

  parameter CFDAeroMapRecord pAero(
    referenceSpeed = 10,
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

  BobLib.Vehicle.Aero.CFDAeroMap aero(pAero = pAero) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  aero.rideHeight_1 = 0.03;
  aero.rideHeight_2 = 0.03;
  aero.rideHeight_3 = 0.03;
  aero.rideHeight_4 = 0.03;
  aero.speed = 20;

  assert(abs(aero.drag - 52.0) < 1e-9, "CFDAeroMap drag interpolation changed");
  assert(abs(aero.downforce - 520.0) < 1e-9, "CFDAeroMap downforce interpolation changed");
  assert(abs(aero.force[1] + aero.drag) < 1e-9, "CFDAeroMap x-force sign changed");
  assert(abs(aero.force[3] + aero.downforce) < 1e-9, "CFDAeroMap z-force sign changed");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestCFDAeroMap;
