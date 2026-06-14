within BobLibVehicleInterfaces.PowerElectronics;

model InverterDC
  "Vehicle-level power-electronics subsystem backed by the internal DC inverter core"
  extends BobLibVehicleInterfaces.Icons.InverterDCIcon;

  import SI = Modelica.Units.SI;

  Modelica.Electrical.Analog.Interfaces.PositivePin p
    "DC bus positive" annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin n
    "DC bus negative" annotation(
      Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput P_req
    "Requested mechanical/electrical output power [W] (+motoring, -regen)" annotation(
      Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

  Modelica.Blocks.Interfaces.RealOutput P_out(start = 0)
    "Electrical power delivered to motor side [W] (+motoring, -regen)" annotation(
      Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = -90),
        iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  parameter Real eta_mot = 0.97 "Inverter efficiency (motoring)";
  parameter Real eta_reg = 0.95 "Inverter efficiency (regen)";
  parameter SI.Voltage V_eps = 1.0 "Small voltage to avoid division by zero";
  parameter Boolean enabled = true "Enable DC conversion";
  parameter SI.Power P_max_mot = 120e3 "Maximum motoring output power";
  parameter SI.Power P_max_reg = 80e3
    "Maximum regenerative input power as positive magnitude";
  parameter SI.Power P_dc_max_mot = 150e3 "Maximum DC discharge power";
  parameter SI.Power P_dc_max_reg = 100e3
    "Maximum DC charge power as positive magnitude";
  parameter SI.Current I_dc_dis_max = 350 "Maximum DC discharge current";
  parameter SI.Current I_dc_chg_max = 250
    "Maximum DC charge current as positive magnitude";
  parameter SI.Voltage V_dc_min = 200 "Minimum bus voltage for motoring";
  parameter SI.Voltage V_dc_max = 720 "Maximum bus voltage for regen";
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

  SI.Voltage V_dc(start = 500) "Measured DC bus voltage";
  SI.Current I_dc(start = 0) "DC current drawn from battery (+discharge)";
  SI.Power P_dc(start = 0) "DC electrical power from battery";
  SI.Power P_loss(start = 0) "Inverter losses";
  SI.Power P_req_limited(start = 0)
    "Power request after bus, current, and nameplate limits";
  SI.Power P_mot_max_active(start = 0) "Active motoring power limit";
  SI.Power P_reg_max_active(start = 0)
    "Active regen power limit as positive magnitude";
  Real eta_eff(unit = "1", start = 0.95) "Active conversion efficiency";
  Real powerFraction(unit = "1", start = 0)
    "Normalized absolute output power request";

protected
  BobLibVehicleInterfaces.PowerElectronics.Internal.InverterDCCore inverter(
    eta_mot = eta_mot,
    eta_reg = eta_reg,
    V_eps = V_eps,
    enabled = enabled,
    P_max_mot = P_max_mot,
    P_max_reg = P_max_reg,
    P_dc_max_mot = P_dc_max_mot,
    P_dc_max_reg = P_dc_max_reg,
    I_dc_dis_max = I_dc_dis_max,
    I_dc_chg_max = I_dc_chg_max,
    V_dc_min = V_dc_min,
    V_dc_max = V_dc_max,
    P_nominal = P_nominal,
    P_standby = P_standby,
    eta_min = eta_min,
    eta_max = eta_max,
    powerFractionTable = powerFractionTable,
    eta_mot_table = eta_mot_table,
    eta_reg_table = eta_reg_table) annotation(
      Placement(transformation(extent = {{-20, -20}, {20, 20}})));

equation
  V_dc = inverter.V_dc;
  I_dc = inverter.I_dc;
  P_dc = inverter.P_dc;
  P_loss = inverter.P_loss;
  P_req_limited = inverter.P_req_limited;
  P_mot_max_active = inverter.P_mot_max_active;
  P_reg_max_active = inverter.P_reg_max_active;
  eta_eff = inverter.eta_eff;
  powerFraction = inverter.powerFraction;

  connect(p, inverter.p) annotation(
    Line(points = {{-100, 0}, {-20, 0}}, color = {0, 0, 255}));
  connect(inverter.n, n) annotation(
    Line(points = {{20, 0}, {100, 0}}, color = {0, 0, 255}));
  connect(P_req, inverter.P_req) annotation(
    Line(points = {{0, 120}, {0, 20}}, color = {0, 0, 127}));
  connect(inverter.P_out, P_out) annotation(
    Line(points = {{0, -20}, {0, -120}}, color = {0, 0, 127}));

  annotation(Documentation(info = "<html>
<p>
Vehicle-level power-electronics adapter for the BobLib DC inverter model.
The public model owns the subsystem boundary used by vehicle experiments;
the conversion equations and electrical current source live in
<code>Internal.InverterDCCore</code>.
</p>
</html>"));
end InverterDC;
