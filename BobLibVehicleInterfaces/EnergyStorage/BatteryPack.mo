within BobLibVehicleInterfaces.EnergyStorage;
model BatteryPack

  "BobLib battery pack exposed through VehicleInterfaces energy-storage pins"
  extends VehicleInterfaces.Icons.Battery;
  extends VehicleInterfaces.EnergyStorage.Interfaces.Base;

  import SI = Modelica.Units.SI;

  parameter Integer Ns(min = 1) = 140 "Battery cells in series";
  parameter Integer Np(min = 1) = 4 "Battery cells in parallel";
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1
    "Initial battery state of charge";

  BobLibVehicleInterfaces.EnergyStorage.Internal.TheveninBatteryPack battery(
    Ns = Ns,
    Np = Np,
    SOC_start = SOC_start) annotation(
      Placement(transformation(origin = {0, 0}, extent = {{-40, -40}, {40, 40}})));

  output Real SOC(unit = "1") "Pack state of charge";
  output Real SOE(unit = "1") "Pack state of energy";
  output SI.Energy E_remaining "Estimated remaining pack energy";
  output SI.Voltage v "Terminal voltage";
  output SI.Current i "Battery current, positive while discharging";
  output SI.Power P "Electrical power, positive while discharging";

protected
  Modelica.Blocks.Sources.RealExpression voltageBusSignal(
    y = v) "Terminal voltage published to batteryBus" annotation(
      Placement(transformation(origin = {-74, -28}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression currentBusSignal(
    y = i) "Terminal current published to batteryBus" annotation(
      Placement(transformation(origin = {-74, -38}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression powerBusSignal(
    y = P) "Terminal power published to batteryBus" annotation(
      Placement(transformation(origin = {-74, -48}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression stateOfChargeBusSignal(
    y = SOC) "State of charge published to batteryBus" annotation(
      Placement(transformation(origin = {-74, -58}, extent = {{-8, -4}, {8, 4}})));
  Modelica.Blocks.Sources.RealExpression stateOfEnergyBusSignal(
    y = SOE) "State of energy published to batteryBus" annotation(
      Placement(transformation(origin = {-74, -68}, extent = {{-8, -4}, {8, 4}})));

equation
  SOC = battery.SOC;
  SOE = battery.SOE;
  E_remaining = battery.E_remaining;
  v = battery.v;
  i = battery.i;
  P = battery.P;

  connect(voltageBusSignal.y, controlBus.batteryBus.voltage) annotation(
    Line(points = {{-65.2, -28}, {-112, -28}, {-112, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(currentBusSignal.y, controlBus.batteryBus.current) annotation(
    Line(points = {{-65.2, -38}, {-114, -38}, {-114, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(powerBusSignal.y, controlBus.batteryBus.power) annotation(
    Line(points = {{-65.2, -48}, {-116, -48}, {-116, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(stateOfChargeBusSignal.y, controlBus.batteryBus.soc) annotation(
    Line(points = {{-65.2, -58}, {-118, -58}, {-118, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(stateOfEnergyBusSignal.y, controlBus.batteryBus.soe) annotation(
    Line(points = {{-65.2, -68}, {-120, -68}, {-120, -60}, {-100, -60}}, color = {0, 0, 127}));

  connect(pin_p, battery.p) annotation(
    Line(points = {{100, 60}, {-40, 60}, {-40, 0}}, color = {0, 0, 255}));
  connect(pin_n, battery.n) annotation(
    Line(points = {{100, -60}, {40, -60}, {40, 0}}, color = {0, 0, 255}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces-facing battery pack backed by the internal Thevenin pack implementation.
It inherits <code>VehicleInterfaces.EnergyStorage.Interfaces.Base</code> and
connects the standard <code>pin_p</code>/<code>pin_n</code> terminals directly
to the BobLib battery terminals. The pack publishes terminal voltage, current,
power, SOC, and SOE measurements on <code>controlBus.batteryBus</code>.
</p>
</html>"));
end BatteryPack;
