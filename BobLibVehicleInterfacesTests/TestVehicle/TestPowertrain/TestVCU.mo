within BobLibVehicleInterfacesTests.TestVehicle.TestPowertrain;

model TestVCU
  BobLibVehicleInterfaces.Controllers.VCU vcu(
    tau_max = 200,
    w_eps = 0.1) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  vcu.cmd_torque_motor = 250;
  vcu.cmd_regen_limit = 50;
  vcu.cmd_inverter_enable = true;
  vcu.sens_motor_speed = 100;
  vcu.sens_hv_bus_voltage = 400;
  vcu.sens_hv_bus_current = 20;

  assert(vcu.vcu_active, "VCU enable path changed");
  assert(abs(vcu.tau_cmd_limited - 200) < 1e-9, "VCU torque limiting changed");
  assert(abs(vcu.P_req - 20000) < 1e-6, "VCU torque-to-power conversion changed");

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestVCU;
