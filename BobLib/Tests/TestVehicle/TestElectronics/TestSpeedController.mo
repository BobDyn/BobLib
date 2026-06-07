within BobLib.Tests.TestVehicle.TestElectronics;

model TestSpeedController
  BobLib.Vehicle.Electronics.Controllers.SpeedController speedController(
    kp = 10,
    ki = 0,
    torque_max = 1000,
    torque_min = -1000,
    tau = 0.02) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  speedController.v = 5;
  speedController.v_ref = 10;

  annotation(
    experiment(StartTime = 0, StopTime = 0.2, Tolerance = 1e-06, Interval = 0.002));
end TestSpeedController;
