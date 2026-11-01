within BobLib.PowerElectronics;

model InverterDC

  "Vehicle-level power-electronics subsystem backed by the internal DC inverter core"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.PowerElectronics.InverterDCRecord pInverter(
    P_max_mot = 120e3,
    P_max_reg = 80e3,
    V_dc_max = 720) "DC power, voltage, current and efficiency calibration" annotation(Dialog(group = "Calibration"));

  extends BobLib.PowerElectronics.Interfaces.Base;

  parameter Boolean enabled = true "Enable DC conversion" annotation(Dialog(group = "Operating settings"), choices(checkBox = true));

  SI.Voltage V_dc(start = 500) "Measured DC bus voltage";
  SI.Current I_dc(start = 0) "DC current drawn from battery (+discharge)";
  SI.Power P_dc(start = 0) "DC electrical power from battery";
  SI.Power P_loss(start = 0) "Inverter losses";
  output SI.Power P_out(start = 0)
    "Electrical power delivered to motor side [W] (+motoring, -regen)";
  output SI.Power P_req(start = 0)
    "Power request received from electricMotorControlBus";
  output SI.Voltage V_motor(start = 500) "Motor-side DC voltage";
  output SI.Current I_motor(start = 0)
    "Motor-side current delivered to the motor positive pin";
  SI.Power P_req_limited(start = 0)
    "Power request after bus, current, and nameplate limits";
  SI.Power P_mot_max_active(start = 0) "Active motoring power limit";
  SI.Power P_reg_max_active(start = 0)
    "Active regen power limit as positive magnitude";
  Real eta_eff(unit = "1", start = 0.95) "Active conversion efficiency";
  Real powerFraction(unit = "1", start = 0)
    "Normalized absolute output power request";

protected
  Modelica.Blocks.Sources.RealExpression availablePowerSignal(y = P_out)
    "Power allocation for the averaged motor load" annotation(
    Placement(transformation(origin = {35, 35}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage motorVoltage
    "Averaged output voltage with a shared DC return" annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  BobLib.PowerElectronics.Internal.InverterDCCore inverter(pInverter = pInverter, enabled = enabled) annotation(
      Placement(transformation(origin = {-40, 0}, extent = {{-20, -20}, {20, 20}})));

  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus
    "Typed electricMotorControlBus signal contract" annotation(
      Placement(transformation(origin = {0, 65}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(electricMotorControlBus, controlBus.electricMotorControlBus) annotation(
    Line(points = {{0, 65}, {0, 100}}, color = {255, 204, 51}, thickness = 0.5));
  P_req = inverter.P_req;
  V_dc = inverter.V_dc;
  I_dc = inverter.I_dc;
  P_dc = inverter.P_dc;
  P_loss = inverter.P_loss;
  P_out = inverter.P_out;
  P_req_limited = inverter.P_req_limited;
  P_mot_max_active = inverter.P_mot_max_active;
  P_reg_max_active = inverter.P_reg_max_active;
  eta_eff = inverter.eta_eff;
  powerFraction = inverter.powerFraction;
  V_motor = motor_p.v - motor_n.v;
  motorVoltage.v = V_dc;
  I_motor = -motor_p.i;

  connect(motorVoltage.p, motor_p) annotation(
    Line(points = {{60, 10}, {60, 70}, {100, 70}}, color = {0, 0, 255}));
  connect(motorVoltage.n, motor_n) annotation(
    Line(points = {{60, -10}, {60, -70}, {100, -70}}, color = {0, 0, 255}));
  connect(n, motor_n) annotation(
    Line(points = {{-100, -70}, {100, -70}}, color = {0, 0, 255}));
  connect(availablePowerSignal.y, electricMotorControlBus.availablePower) annotation(
    Line(points = {{24, 35}, {0, 35}, {0, 65}}, color = {0, 0, 127}));

  connect(p, inverter.p) annotation(
    Line(points = {{-100, 70}, {-80, 70}, {-80, 0}, {-60, 0}}, color = {0, 0, 255}));
  connect(inverter.n, n) annotation(
    Line(points = {{-20, 0}, {-20, -70}, {-100, -70}}, color = {0, 0, 255}));

  connect(electricMotorControlBus.powerRequest, inverter.P_req) annotation(
    Line(points = {{0, 65}, {-40, 65}, {-40, 24}}, color = {0, 0, 127}));
  annotation(Documentation(info = "<html>
<p>
Vehicle-level power-electronics adapter for the BobLib DC inverter model.
The public model owns the subsystem boundary used by vehicle experiments;
the conversion equations and electrical current source live in
<code>Internal.InverterDCCore</code>. The inverter subscribes to
<code>controlBus.electricMotorControlBus.powerRequest</code> instead of
receiving a direct controller pin. It publishes the limited allocation as
<code>electricMotorControlBus.availablePower</code> and supplies an averaged DC
voltage. A compatible averaged motor consumes this allocation through its own
current law. The electrical return is shared with the input DC bus. An arbitrary
load that ignores the allocation does not implement this averaged power contract.
</p>
</html>"),
  Icon,
  Diagram(coordinateSystem(preserveAspectRatio = true)));
end InverterDC;
