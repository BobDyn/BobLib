within BobLibVehicleInterfaces.EnergyStorage.Internal;

model TheveninBatteryPack "Table-driven Thevenin battery pack"
  extends BobLibVehicleInterfaces.Icons.BatteryPackIcon;

  extends BobLibVehicleInterfaces.EnergyStorage.Internal.BatteryBase;

  import SI = Modelica.Units.SI;
  import Modelica.Math.Vectors.interpolate;

  // Cell parameters. R_cell and E_cell are kept for compatibility with older
  // fixtures; the table defaults are derived from them.
  parameter SI.Resistance R_cell = 0.003 "Nominal cell DC resistance";
  parameter SI.Energy E_cell = 15e3 "Nominal cell energy";
  parameter SI.Voltage V_cell_nominal = 3.7 "Nominal cell voltage used to derive charge capacity";
  parameter SI.ElectricCharge Q_cell = E_cell/V_cell_nominal "Cell charge capacity";
  parameter Real eta_charge_coulombic(unit = "1", min = 0, max = 1) = 0.995 "Coulombic charge efficiency";
  parameter SI.Voltage V_cell_min = 2.8 "Minimum loaded cell voltage";
  parameter SI.Voltage V_cell_max = 4.2 "Maximum charge cell voltage";

  // OCV curve (cell-level)
  parameter Real SOC_table[:] = {0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
  parameter Real SOE_table[:] = {0, 0.07, 0.16, 0.36, 0.57, 0.79, 1.0}
    "Remaining energy fraction at each SOC breakpoint";
  parameter SI.Voltage V_ocv_cell_table[:] = {3.0, 3.3, 3.5, 3.7, 3.85, 4.0, 4.2};
  parameter SI.Resistance R0_dis_cell_table[:] = {
    1.60*R_cell, 1.25*R_cell, 1.05*R_cell, R_cell, R_cell, 1.08*R_cell, 1.25*R_cell}
    "Ohmic discharge resistance vs SOC";
  parameter SI.Resistance R0_chg_cell_table[:] = {
    1.80*R_cell, 1.35*R_cell, 1.10*R_cell, 1.03*R_cell, 1.05*R_cell, 1.18*R_cell, 1.45*R_cell}
    "Ohmic charge resistance vs SOC";
  parameter SI.Resistance R1_dis_cell_table[:] = {
    0.90*R_cell, 0.70*R_cell, 0.55*R_cell, 0.45*R_cell, 0.45*R_cell, 0.55*R_cell, 0.75*R_cell}
    "Polarization discharge resistance vs SOC";
  parameter SI.Resistance R1_chg_cell_table[:] = {
    1.00*R_cell, 0.80*R_cell, 0.60*R_cell, 0.50*R_cell, 0.52*R_cell, 0.68*R_cell, 0.90*R_cell}
    "Polarization charge resistance vs SOC";
  parameter SI.Time tau1_dis_table[:] = {18, 15, 12, 10, 10, 12, 16}
    "Polarization discharge time constant vs SOC";
  parameter SI.Time tau1_chg_table[:] = {22, 18, 14, 12, 12, 15, 20}
    "Polarization charge time constant vs SOC";
  parameter SI.Current I_dis_cell_max_table[:] = {15, 35, 55, 75, 80, 70, 45}
    "Cell discharge current limit vs SOC";
  parameter SI.Current I_chg_cell_max_table[:] = {35, 45, 45, 40, 35, 25, 8}
    "Cell charge current limit vs SOC";

  final parameter SI.Resistance R_pack = (Ns/Np)*R_cell "Nominal pack resistance";
  final parameter SI.Energy E_pack = Ns*Np*E_cell "Nominal pack energy";
  final parameter SI.ElectricCharge Q_pack = Np*Q_cell "Pack charge capacity";

  Modelica.Electrical.Analog.Sources.SignalVoltage terminalVoltage annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  // Diagnostics and limit estimates
  SI.Voltage v_oc_cell "Cell open-circuit voltage";
  SI.Voltage v_oc_pack "Pack open-circuit voltage";
  SI.Voltage v_rc(start = 0, fixed = true) "Polarization voltage drop";
  SI.Resistance R0_pack "Active pack ohmic resistance";
  SI.Resistance R1_pack "Active pack polarization resistance";
  SI.Current I_dis_max "Available discharge current limit";
  SI.Current I_chg_max "Available charge current limit as positive magnitude";
  SI.Power P_dis_max "Estimated available discharge power";
  SI.Power P_chg_max "Estimated available charge power as positive magnitude";
  SI.Power P_loss_ohmic "Ohmic heat generation";
  SI.Power P_loss_polarization "Polarization heat generation";
  SI.Power P_loss "Total estimated cell electrical loss";
  SI.Power P_chemical "Open-circuit chemical power draw";

protected
  parameter SI.Resistance R_eps = 1e-6 "Resistance floor";
  parameter SI.Time tau_eps = 1e-6 "Time-constant floor";
  Real SOC_lookup(unit = "1");
  SI.Resistance R0_cell;
  SI.Resistance R1_cell;
  SI.Time tau1;
  SI.Voltage V_pack_min;
  SI.Voltage V_pack_max;
  SI.Current I_dis_table_limit;
  SI.Current I_chg_table_limit;
  SI.Current I_dis_voltage_limit;
  SI.Current I_chg_voltage_limit;
  SI.Current i_soc;

equation
  assert(Ns > 0 and Np > 0, "BatteryPack: Ns and Np must be >= 1");
  assert(V_cell_nominal > 0, "BatteryPack: V_cell_nominal must be positive");
  assert(Q_cell > 0, "BatteryPack: Q_cell must be positive");
  assert(size(SOC_table, 1) >= 2, "BatteryPack: SOC_table must contain at least two breakpoints");
  assert(size(SOE_table, 1) == size(SOC_table, 1), "BatteryPack: SOE_table size must match SOC_table");
  assert(size(V_ocv_cell_table, 1) == size(SOC_table, 1), "BatteryPack: V_ocv_cell_table size must match SOC_table");
  assert(size(R0_dis_cell_table, 1) == size(SOC_table, 1), "BatteryPack: R0_dis_cell_table size must match SOC_table");
  assert(size(R0_chg_cell_table, 1) == size(SOC_table, 1), "BatteryPack: R0_chg_cell_table size must match SOC_table");
  assert(size(R1_dis_cell_table, 1) == size(SOC_table, 1), "BatteryPack: R1_dis_cell_table size must match SOC_table");
  assert(size(R1_chg_cell_table, 1) == size(SOC_table, 1), "BatteryPack: R1_chg_cell_table size must match SOC_table");
  assert(size(tau1_dis_table, 1) == size(SOC_table, 1), "BatteryPack: tau1_dis_table size must match SOC_table");
  assert(size(tau1_chg_table, 1) == size(SOC_table, 1), "BatteryPack: tau1_chg_table size must match SOC_table");
  assert(size(I_dis_cell_max_table, 1) == size(SOC_table, 1), "BatteryPack: I_dis_cell_max_table size must match SOC_table");
  assert(size(I_chg_cell_max_table, 1) == size(SOC_table, 1), "BatteryPack: I_chg_cell_max_table size must match SOC_table");

  // Cell lookup values are clamped to the characterized SOC window.
  SOC_lookup = noEvent(min(max(SOC, SOC_table[1]), SOC_table[size(SOC_table, 1)]));
  v_oc_pack = Ns*v_oc_cell;
  v_oc_cell = interpolate(SOC_table, V_ocv_cell_table, SOC_lookup);
  R0_cell = if noEvent(i >= 0) then
    interpolate(SOC_table, R0_dis_cell_table, SOC_lookup)
  else
    interpolate(SOC_table, R0_chg_cell_table, SOC_lookup);
  R1_cell = if noEvent(i >= 0) then
    interpolate(SOC_table, R1_dis_cell_table, SOC_lookup)
  else
    interpolate(SOC_table, R1_chg_cell_table, SOC_lookup);
  tau1 = if noEvent(i >= 0) then
    max(interpolate(SOC_table, tau1_dis_table, SOC_lookup), tau_eps)
  else
    max(interpolate(SOC_table, tau1_chg_table, SOC_lookup), tau_eps);
  R0_pack = (Ns/Np)*max(R0_cell, R_eps);
  R1_pack = (Ns/Np)*max(R1_cell, R_eps);
  V_pack_min = Ns*V_cell_min;
  V_pack_max = Ns*V_cell_max;

  I_dis_table_limit = Np*interpolate(SOC_table, I_dis_cell_max_table, SOC_lookup);
  I_chg_table_limit = Np*interpolate(SOC_table, I_chg_cell_max_table, SOC_lookup);
  I_dis_voltage_limit = noEvent(max(0, (v_oc_pack - v_rc - V_pack_min)/R0_pack));
  I_chg_voltage_limit = noEvent(max(0, (V_pack_max - (v_oc_pack - v_rc))/R0_pack));
  I_dis_max = noEvent(min(I_dis_table_limit, I_dis_voltage_limit));
  I_chg_max = noEvent(min(I_chg_table_limit, I_chg_voltage_limit));
  P_dis_max = noEvent(max(0, (v_oc_pack - v_rc - I_dis_max*R0_pack)*I_dis_max));
  P_chg_max = noEvent(max(0, (v_oc_pack - v_rc + I_chg_max*R0_pack)*I_chg_max));

  // Thevenin behavior: terminal voltage equals OCV minus ohmic and dynamic
  // polarization drops. Current i is positive while discharging.
  terminalVoltage.v = v_oc_pack - v_rc - i*R0_pack;
  der(v_rc) = (i*R1_pack - v_rc)/tau1;
  P_loss_ohmic = i*i*R0_pack;
  P_loss_polarization = v_rc*v_rc/R1_pack;
  P_loss = P_loss_ohmic + P_loss_polarization;
  P_chemical = i*v_oc_pack;
  SOE = noEvent(min(max(interpolate(SOC_table, SOE_table, SOC_lookup), 0), 1));
  E_remaining = SOE*E_pack;

  // SOC dynamics (bounded). Charge current is adjusted by coulombic efficiency.
  i_soc = if noEvent(i >= 0) then i else eta_charge_coulombic*i;
  if noEvent(SOC_state <= 0 and i_soc > 0) then
    der(SOC_state) = 0;
  elseif noEvent(SOC_state >= 1 and i_soc < 0) then
    der(SOC_state) = 0;
  else
    der(SOC_state) = -i_soc / Q_pack;
  end if;

  connect(p, terminalVoltage.p) annotation(
    Line(points = {{-100, 0}, {-10, 0}}, color = {0, 0, 255}));
  connect(terminalVoltage.n, n) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Model <code>TheveninBatteryPack</code> implements a table-driven Thevenin-equivalent HV battery pack.
</p>
<p>
It tracks state of charge, terminal voltage, bus current, open-circuit voltage, and internal resistance behavior used by the public energy-storage adapter.
</p>
</html>"));
end TheveninBatteryPack;
