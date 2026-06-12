within BobLib.Vehicle.Electronics.PowerElectronics;

model InverterDC "DC-side inverter/load model with efficiency and bus limits"
  import Modelica.SIunits;
  import Modelica.Math.Vectors.interpolate;

  // Electrical ports
  Modelica.Electrical.Analog.Interfaces.PositivePin p "DC bus positive" annotation(
    Placement(transformation(origin={-100,0}, extent={{-10,-10},{10,10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "DC bus negative" annotation(
    Placement(transformation(origin={100,0}, extent={{-10,-10},{10,10}})));

  // Control input
  Modelica.Blocks.Interfaces.RealInput P_req "Requested mechanical/electrical output power [W] (+motoring, −regen)" annotation(
    Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation=-90)));

  // Output
  Modelica.Blocks.Interfaces.RealOutput P_out(start = 0) "Electrical power delivered to motor side [W] (+motoring, −regen)" annotation(
    Placement(transformation(origin = {0, -120}, extent={{-20,-20},{20,20}}, rotation = -90), iconTransformation(origin={0,-110}, extent={{-10,-10},{10,10}}, rotation=-90)));

  // Parameters
  parameter Real eta_mot = 0.97 "Inverter efficiency (motoring)";
  parameter Real eta_reg = 0.95 "Inverter efficiency (regen)";
  parameter SIunits.Voltage V_eps = 1.0 "Small voltage to avoid division by zero";
  parameter Boolean enabled = true "Enable DC conversion";
  parameter SIunits.Power P_max_mot = 120e3 "Maximum motoring output power";
  parameter SIunits.Power P_max_reg = 80e3 "Maximum regenerative input power as positive magnitude";
  parameter SIunits.Power P_dc_max_mot = 150e3 "Maximum DC discharge power";
  parameter SIunits.Power P_dc_max_reg = 100e3 "Maximum DC charge power as positive magnitude";
  parameter SIunits.Current I_dc_dis_max = 350 "Maximum DC discharge current";
  parameter SIunits.Current I_dc_chg_max = 250 "Maximum DC charge current as positive magnitude";
  parameter SIunits.Voltage V_dc_min = 200 "Minimum bus voltage for motoring";
  parameter SIunits.Voltage V_dc_max = 720 "Maximum bus voltage for regen";
  parameter SIunits.Power P_nominal = 100e3 "Power used to normalize efficiency lookup";
  parameter SIunits.Power P_standby = 50 "Enabled standby loss";
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

  // Internal current source
  Modelica.Electrical.Analog.Sources.SignalCurrent I_dc_source annotation(Placement(transformation(origin={0,0}, extent={{-10,-10},{10,10}})));

  // Outputs
  SIunits.Voltage V_dc(start = 500) "Measured DC bus voltage";
  SIunits.Current I_dc(start = 0) "DC current drawn from battery (+discharge)";
  SIunits.Power P_dc(start = 0) "DC electrical power from battery";
  SIunits.Power P_loss(start = 0) "Inverter losses";
  SIunits.Power P_req_limited(start = 0) "Power request after bus, current, and nameplate limits";
  SIunits.Power P_mot_max_active(start = 0) "Active motoring power limit";
  SIunits.Power P_reg_max_active(start = 0) "Active regen power limit as positive magnitude";
  Real eta_eff(unit = "1", start = 0.95) "Active conversion efficiency";
  Real powerFraction(unit = "1", start = 0) "Normalized absolute output power request";

protected
  SIunits.Power P_dc_cmd(start = 0);
  SIunits.Power P_req_nameplate(start = 0);
  SIunits.Power P_mot_from_dc(start = 0);
  SIunits.Power P_regen_dc_sink(start = 0);
  SIunits.Power P_standby_active(start = 0);
  SIunits.Voltage V_abs(start = 500);
  Real eta_mot_eff(unit = "1", start = 0.95);
  Real eta_reg_eff(unit = "1", start = 0.95);

equation
  assert(P_max_mot >= 0 and P_max_reg >= 0, "InverterDC: power limits must be non-negative");
  assert(P_dc_max_mot >= 0 and P_dc_max_reg >= 0, "InverterDC: DC power limits must be non-negative");
  assert(I_dc_dis_max >= 0 and I_dc_chg_max >= 0, "InverterDC: DC current limits must be non-negative");
  assert(P_nominal > 0, "InverterDC: P_nominal must be positive");
  assert(eta_min > 0 and eta_max <= 1 and eta_min <= eta_max, "InverterDC: invalid efficiency bounds");
  assert(size(powerFractionTable, 1) >= 2, "InverterDC: powerFractionTable must contain at least two breakpoints");
  assert(size(eta_mot_table, 1) == size(powerFractionTable, 1), "InverterDC: eta_mot_table size must match powerFractionTable");
  assert(size(eta_reg_table, 1) == size(powerFractionTable, 1), "InverterDC: eta_reg_table size must match powerFractionTable");

  // DC bus measurement
  V_dc = p.v - n.v;
  V_abs = sqrt(V_dc*V_dc + V_eps*V_eps);

  // Clamp the request against nameplate and active electrical limits.
  P_req_nameplate = noEvent(max(min(P_req, P_max_mot), -P_max_reg));
  powerFraction = noEvent(min(max(abs(P_req_nameplate)/P_nominal, powerFractionTable[1]), powerFractionTable[size(powerFractionTable, 1)]));
  eta_mot_eff = noEvent(min(max(interpolate(powerFractionTable, eta_mot_table, powerFraction), eta_min), eta_max));
  eta_reg_eff = noEvent(min(max(interpolate(powerFractionTable, eta_reg_table, powerFraction), eta_min), eta_max));
  eta_eff = if noEvent(P_req_nameplate >= 0) then eta_mot_eff else eta_reg_eff;
  P_standby_active = if enabled then P_standby else 0;

  P_mot_from_dc = noEvent(max(0, min(P_dc_max_mot, V_abs*I_dc_dis_max) - P_standby_active)*eta_mot_eff);
  P_mot_max_active = noEvent(if enabled and V_dc >= V_dc_min then min(P_max_mot, P_mot_from_dc) else 0);
  P_regen_dc_sink = noEvent(min(P_dc_max_reg, V_abs*I_dc_chg_max));
  P_reg_max_active = noEvent(if enabled and V_dc <= V_dc_max then min(P_max_reg, max(0, (P_regen_dc_sink + P_standby_active)/eta_reg_eff)) else 0);
  P_out = if noEvent(P_req_nameplate >= 0) then
    noEvent(min(P_req_nameplate, P_mot_max_active))
  else
    noEvent(max(P_req_nameplate, -P_reg_max_active));
  P_req_limited = P_out;

  // Drive: draw more DC power than delivered on the AC/motor side.
  // Regen: return less DC power than absorbed, with standby loss subtracted.
  P_dc_cmd = if noEvent(P_out >= 0) then
    P_out/eta_mot_eff + P_standby_active
  else
    P_out*eta_reg_eff + P_standby_active;

  // DC current command
  I_dc = P_dc_cmd / V_abs;
  I_dc_source.i = I_dc;

  // Power computation
  P_dc = V_dc * I_dc;
  P_loss = P_dc - P_out;

  connect(p, I_dc_source.p) annotation(
    Line(points = {{-100, 0}, {-10, 0}}, color = {0, 0, 255}));
  connect(I_dc_source.n, n) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));
  annotation(
    Icon(coordinateSystem(extent = {{-100, -70}, {100, 70}}), graphics = {
      Rectangle(extent = {{-72, 46}, {72, -46}}, lineColor = {35, 35, 35}, fillColor = {240, 243, 248}, fillPattern = FillPattern.Solid),
      Line(points = {{-54, -24}, {-18, 24}, {18, -24}, {54, 24}}, color = {30, 90, 150}, thickness = 1),
      Line(points = {{-96, 0}, {-72, 0}}, color = {0, 0, 255}),
      Line(points = {{72, 0}, {96, 0}}, color = {0, 0, 255}),
      Text(extent = {{-66, 64}, {66, 44}}, textString = "%name", lineColor = {32, 32, 32}),
      Text(extent = {{-42, 10}, {42, -10}}, textString = "DC/AC", lineColor = {35, 35, 35})}));
end InverterDC;
