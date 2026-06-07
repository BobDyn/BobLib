within BobLib.Tests.TestVehicle.TestElectronics;

model TestCurvatureController
  BobLib.Vehicle.Electronics.Controllers.CurvatureController curvatureController(
    kp = 1,
    ki = 0,
    activation_time = 0,
    activation_duration = 0.01,
    default_output = 0,
    vx_min = 0.1,
    Tr = 0.01,
    T_rack = 0.02) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  curvatureController.yaw_rate = 0.2;
  curvatureController.v = 10;
  curvatureController.kappa_ref = 0.03;

  annotation(
    experiment(StartTime = 0, StopTime = 0.2, Tolerance = 1e-06, Interval = 0.002));
end TestCurvatureController;
