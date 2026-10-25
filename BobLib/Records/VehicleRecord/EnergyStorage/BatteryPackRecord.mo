within BobLib.Records.VehicleRecord.EnergyStorage;

record BatteryPackRecord

  import Modelica.Units.SI;

  parameter Integer Ns(min = 1) = 140 "Battery cells in series";
  parameter Integer Np(min = 1) = 4 "Battery cells in parallel";
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1
    "Initial battery state of charge";

  parameter SI.Resistance R_cell = 0.003 "Nominal cell DC resistance";
  parameter SI.Energy E_cell = 15e3 "Nominal cell energy";
  parameter SI.Voltage V_cell_nominal = 3.7 "Nominal cell voltage used to derive charge capacity";
  parameter SI.ElectricCharge Q_cell = E_cell/V_cell_nominal "Cell charge capacity";
  parameter Real eta_charge_coulombic(unit = "1", min = 0, max = 1) = 0.995 "Coulombic charge efficiency";
  parameter SI.Voltage V_cell_min = 2.8 "Minimum loaded cell voltage";
  parameter SI.Voltage V_cell_max = 4.2 "Maximum charge cell voltage";
  parameter Real SOC_table[:] = {0, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0} "State-of-charge breakpoints, from zero to one, paired with V_ocv_cell_table";
  parameter Real SOE_table[:] = {0, 0.07, 0.16, 0.36, 0.57, 0.79, 1.0}
    "Remaining energy fraction at each SOC breakpoint";
  parameter SI.Voltage V_ocv_cell_table[:] = {3.0, 3.3, 3.5, 3.7, 3.85, 4.0, 4.2} "Open-circuit voltage per cell at each SOC_table breakpoint";
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
  annotation(
    Documentation(info = "<html>
<p>
Record <code>BatteryPackRecord</code> contains the vehicle-level parameters
passed to <code>EnergyStorage.BatteryPack</code>.
</p>
</html>"));
end BatteryPackRecord;
