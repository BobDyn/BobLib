within BobLib.PowerElectronics.Internal;

model InverterDCCore "DC-side inverter/load physics with efficiency and bus limits"

  import Modelica.Units.SI;
  import Modelica.Math.Vectors.interpolate;

  parameter BobLib.Records.VehicleRecord.PowerElectronics.InverterDCRecord pInverter(
    P_max_mot = 120e3,
    P_max_reg = 80e3,
    V_dc_max = 720) "DC power, voltage, current and efficiency calibration" annotation(Dialog(group = "Calibration"));

  extends BobLib.Icons.InverterDCIcon;

  // Electrical ports
  Modelica.Electrical.Analog.Interfaces.PositivePin p "DC bus positive" annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "DC bus negative" annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  // Control input
  Modelica.Blocks.Interfaces.RealInput P_req "Requested mechanical/electrical output power [W], positive motoring and negative regen" annotation(
    Placement(
      transformation(origin = {-100, 50}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

  // Output
  Modelica.Blocks.Interfaces.RealOutput P_out(start = 0) "Electrical power delivered to motor side [W], positive motoring and negative regen" annotation(
    Placement(
      transformation(origin = {100, 50}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  // Parameters

  parameter Boolean enabled = true "Enable DC conversion" annotation(Dialog(group = "Operating settings"), choices(checkBox = true));

  // Internal current source
  Modelica.Electrical.Analog.Sources.SignalCurrent I_dc_source annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression dcCurrentCommand(
    y = I_dc) annotation(
      Placement(transformation(origin = {-11.3, 30}, extent = {{-8, -8}, {8, 8}})));

  // Outputs
  SI.Voltage V_dc(start = 500) "Measured DC bus voltage";
  SI.Current I_dc(start = 0) "DC current drawn from battery (+discharge)";
  SI.Power P_dc(start = 0) "DC electrical power from battery";
  SI.Power P_loss(start = 0) "Inverter losses";
  SI.Power P_req_limited(start = 0) "Power request after bus, current, and nameplate limits";
  SI.Power P_mot_max_active(start = 0) "Active motoring power limit";
  SI.Power P_reg_max_active(start = 0) "Active regen power limit as positive magnitude";
  Real eta_eff(unit = "1", start = 0.95) "Active conversion efficiency";
  Real powerFraction(unit = "1", start = 0) "Normalized absolute output power request";

protected
  SI.Power P_dc_cmd(start = 0);
  SI.Power P_req_nameplate(start = 0);
  SI.Power P_mot_from_dc(start = 0);
  SI.Power P_regen_dc_sink(start = 0);
  SI.Power P_standby_active(start = 0);
  SI.Voltage V_abs(start = 500);
  Real eta_mot_eff(unit = "1", start = 0.95);
  Real eta_reg_eff(unit = "1", start = 0.95);

initial equation
  assert(pInverter.V_eps > 0 and pInverter.V_dc_min >= 0 and pInverter.V_dc_max > pInverter.V_dc_min,
    "InverterDC: voltage regularization must be positive and bus voltage limits ordered");
  assert(pInverter.P_standby >= 0, "InverterDC: standby loss must be non-negative");
  for i in 2:size(pInverter.powerFractionTable, 1) loop
    assert(pInverter.powerFractionTable[i] > pInverter.powerFractionTable[i - 1],
      "InverterDC: powerFractionTable breakpoints must be strictly increasing");
  end for;

equation
  assert(pInverter.P_max_mot >= 0 and pInverter.P_max_reg >= 0, "InverterDC: power limits must be non-negative");
  assert(
    pInverter.P_dc_max_mot >= 0 and pInverter.P_dc_max_reg >= 0,
    "InverterDC: DC power limits must be non-negative");
  assert(
    pInverter.I_dc_dis_max >= 0 and pInverter.I_dc_chg_max >= 0,
    "InverterDC: DC current limits must be non-negative");
  assert(pInverter.P_nominal > 0, "InverterDC: P_nominal must be positive");
  assert(
    pInverter.eta_min > 0 and pInverter.eta_max <= 1 and pInverter.eta_min <= pInverter.eta_max,
    "InverterDC: invalid efficiency bounds");
  assert(
    size(pInverter.powerFractionTable, 1) >= 2,
    "InverterDC: powerFractionTable must contain at least two breakpoints");
  assert(
    size(pInverter.eta_mot_table, 1) == size(pInverter.powerFractionTable, 1),
    "InverterDC: eta_mot_table size must match powerFractionTable");
  assert(
    size(pInverter.eta_reg_table, 1) == size(pInverter.powerFractionTable, 1),
    "InverterDC: eta_reg_table size must match powerFractionTable");

  // DC bus measurement
  V_dc = p.v - n.v;
  V_abs = sqrt(V_dc*V_dc + pInverter.V_eps*pInverter.V_eps);

  // Clamp the request against nameplate and active electrical limits.
  P_req_nameplate = noEvent(max(min(P_req, pInverter.P_max_mot), -pInverter.P_max_reg));
  powerFraction = noEvent(min(max(abs(P_req_nameplate)/pInverter.P_nominal, pInverter.powerFractionTable[1]), pInverter.powerFractionTable[size(pInverter.powerFractionTable, 1)]));
  eta_mot_eff = noEvent(min(max(interpolate(pInverter.powerFractionTable, pInverter.eta_mot_table, powerFraction), pInverter.eta_min), pInverter.eta_max));
  eta_reg_eff = noEvent(min(max(interpolate(pInverter.powerFractionTable, pInverter.eta_reg_table, powerFraction), pInverter.eta_min), pInverter.eta_max));
  eta_eff = if noEvent(P_req_nameplate >= 0) then eta_mot_eff else eta_reg_eff;
  P_standby_active = if enabled then pInverter.P_standby else 0;

  P_mot_from_dc = noEvent(max(0, min(pInverter.P_dc_max_mot, V_abs*pInverter.I_dc_dis_max) - P_standby_active)*eta_mot_eff);
  P_mot_max_active = noEvent(if enabled and V_dc >= pInverter.V_dc_min then min(pInverter.P_max_mot, P_mot_from_dc) else 0);
  P_regen_dc_sink = noEvent(min(pInverter.P_dc_max_reg, V_abs*pInverter.I_dc_chg_max));
  P_reg_max_active = noEvent(if enabled and V_dc <= pInverter.V_dc_max then min(pInverter.P_max_reg, max(0, (P_regen_dc_sink + P_standby_active)/eta_reg_eff)) else 0);
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

  // Power computation
  P_dc = V_dc * I_dc;
  P_loss = P_dc - P_out;

  connect(dcCurrentCommand.y, I_dc_source.i) annotation(
    Line(points = {{-2.5, 30}, {8, 30}, {8, 20}, {0, 20}, {0, 12}}, color = {0, 0, 127}));
  connect(p, I_dc_source.p) annotation(
    Line(points = {{-100, 0}, {-10, 0}}, color = {0, 0, 255}));
  connect(I_dc_source.n, n) annotation(Line(points = {{10, 0}, {100, 0}}, color = {0, 0, 255}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>InverterDCCore</code> implements the DC-side inverter/load core with efficiency and bus limits.
</p>
<p>
It converts the VCU power request into DC bus current demand, respects enable state, applies efficiency behavior, and supplies the internal physics behind the public inverter adapter.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}}, preserveAspectRatio = true)));
end InverterDCCore;
