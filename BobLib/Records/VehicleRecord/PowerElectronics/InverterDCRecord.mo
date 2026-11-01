within BobLib.Records.VehicleRecord.PowerElectronics;

record InverterDCRecord

  import Modelica.Units.SI;

  parameter Real eta_mot = 0.97 "Inverter efficiency (motoring)";
  parameter Real eta_reg = 0.95 "Inverter efficiency (regen)";
  parameter SI.Power P_max_mot = 124e3 "Maximum motoring output power";
  parameter SI.Power P_max_reg = 124e3
    "Maximum regenerative input power as positive magnitude";
  parameter SI.Voltage V_dc_max = 588
    "Maximum DC bus voltage before regen is disabled";

  parameter SI.Voltage V_eps = 1.0 "Small voltage to avoid division by zero";
  parameter SI.Power P_dc_max_mot = 150e3 "Maximum DC discharge power";
  parameter SI.Power P_dc_max_reg = 100e3
    "Maximum DC charge power as positive magnitude";
  parameter SI.Current I_dc_dis_max = 350 "Maximum DC discharge current";
  parameter SI.Current I_dc_chg_max = 250
    "Maximum DC charge current as positive magnitude";
  parameter SI.Voltage V_dc_min = 200 "Minimum bus voltage for motoring";
  parameter SI.Power P_nominal = 100e3
    "Power used to normalize efficiency lookup";
  parameter SI.Power P_standby = 50 "Enabled standby loss";
  parameter Real eta_min(unit = "1") = 0.2 "Efficiency floor";
  parameter Real eta_max(unit = "1") = 0.995 "Efficiency ceiling";
  parameter Real powerFractionTable[:] = {0, 0.05, 0.15, 0.35, 0.65, 1.0}
    "Efficiency map breakpoints by abs(P_out)/P_nominal";
  parameter Real eta_mot_table[:] = {
    max(eta_min, eta_mot - 0.12),
    max(eta_min, eta_mot - 0.06),
    max(eta_min, eta_mot - 0.025),
    max(eta_min, eta_mot - 0.010),
    eta_mot,
    max(eta_min, eta_mot - 0.005)}
    "Motoring efficiency vs normalized output power";
  parameter Real eta_reg_table[:] = {
    max(eta_min, eta_reg - 0.12),
    max(eta_min, eta_reg - 0.06),
    max(eta_min, eta_reg - 0.025),
    max(eta_min, eta_reg - 0.010),
    eta_reg,
    max(eta_min, eta_reg - 0.005)}
    "Regen efficiency vs normalized output power";
  annotation(
    Documentation(info = "<html>
<p>
Record <code>InverterDCRecord</code> contains the vehicle-level parameters
passed to <code>PowerElectronics.InverterDC</code>.
</p>
<p>
Current, power, voltage, and efficiency-map calibration is passed as this complete
record from the vehicle definition to the inverter adapter and its core.

</p>
</html>"));
end InverterDCRecord;
