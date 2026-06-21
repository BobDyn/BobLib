within BobLib.Vehicle.Electronics;

model ElectronicsAssembly

  "VCU and inverter assembly for vehicle-level ELC substitution"
  extends BobLib.Vehicle.Electronics.ElectronicsBase;

  BobLib.Vehicle.Electronics.Controls.VCU vcu(
    tau_max = tau_max,
    w_eps = w_eps,
    motorSpeedSign = motorSpeedSign) annotation(
      Placement(transformation(origin = {-34, -18}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Vehicle.Electronics.PowerElectronics.InverterDC inverter(
    P_max_mot = inverterPMaxMot,
    P_max_reg = inverterPMaxReg,
    V_dc_max = inverterVdcMax) annotation(
      Placement(transformation(origin = {38, -18}, extent = {{-10, -10}, {10, 10}})));

equation
  vcu.cmd_torque_motor = cmd_torque_motor;
  vcu.cmd_regen_limit = cmd_regen_limit;
  vcu.cmd_inverter_enable = cmd_inverter_enable;
  vcu.sens_motor_speed = sens_motor_speed;
  vcu.sens_hv_bus_voltage = inverter.V_dc;
  vcu.sens_hv_bus_current = inverter.I_dc;

  inverter.P_req = vcu.P_req;

  P_motor = inverter.P_out;
  V_dc = inverter.V_dc;
  I_dc = inverter.I_dc;
  P_req = vcu.P_req;
  tau_cmd_limited = vcu.tau_cmd_limited;
  active = vcu.vcu_active;

  connect(p, inverter.p) annotation(
    Line(points = {{-100, 40}, {18, 40}, {18, -18}, {28, -18}}, color = {0, 0, 255}));
  connect(inverter.n, n) annotation(
    Line(points = {{48, -18}, {62, -18}, {62, -40}, {-100, -40}}, color = {0, 0, 255}));

  annotation(
    Diagram(coordinateSystem(extent = {{-120, -100}, {120, 100}})));
end ElectronicsAssembly;
