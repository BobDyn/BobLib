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
  Modelica.Blocks.Interfaces.RealOutput voltageBusSignal(
    quantity = "ElectricPotential",
    unit = "V") "Terminal voltage published to batteryBus";
  Modelica.Blocks.Interfaces.RealOutput currentBusSignal(
    quantity = "ElectricCurrent",
    unit = "A") "Terminal current published to batteryBus";
  Modelica.Blocks.Interfaces.RealOutput powerBusSignal(
    quantity = "Power",
    unit = "W") "Terminal power published to batteryBus";
  Modelica.Blocks.Interfaces.RealOutput stateOfChargeBusSignal(unit = "1")
    "State of charge published to batteryBus";
  Modelica.Blocks.Interfaces.RealOutput stateOfEnergyBusSignal(unit = "1")
    "State of energy published to batteryBus";

equation
  SOC = battery.SOC;
  SOE = battery.SOE;
  E_remaining = battery.E_remaining;
  v = battery.v;
  i = battery.i;
  P = battery.P;
  voltageBusSignal = v;
  currentBusSignal = i;
  powerBusSignal = P;
  stateOfChargeBusSignal = SOC;
  stateOfEnergyBusSignal = SOE;

  connect(voltageBusSignal, controlBus.batteryBus.voltage) annotation(
    Line(points = {{0, 0}, {-118, 0}, {-118, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(currentBusSignal, controlBus.batteryBus.current) annotation(
    Line(points = {{0, 0}, {-116, 0}, {-116, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(powerBusSignal, controlBus.batteryBus.power) annotation(
    Line(points = {{0, 0}, {-114, 0}, {-114, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(stateOfChargeBusSignal, controlBus.batteryBus.soc) annotation(
    Line(points = {{0, 0}, {-112, 0}, {-112, -60}, {-100, -60}}, color = {0, 0, 127}));
  connect(stateOfEnergyBusSignal, controlBus.batteryBus.soe) annotation(
    Line(points = {{0, 0}, {-110, 0}, {-110, -60}, {-100, -60}}, color = {0, 0, 127}));

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
