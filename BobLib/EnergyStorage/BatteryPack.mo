within BobLib.EnergyStorage;
model BatteryPack

  "BobLib battery pack exposed through VehicleInterfaces energy-storage pins"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.EnergyStorage.BatteryPackRecord pBattery "Cell topology, electrical maps and initial charge" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.Battery;

  extends BobLib.EnergyStorage.Interfaces.Base(final includeHeatPort = false);

  BobLib.EnergyStorage.Internal.TheveninBatteryPack battery(pBattery = pBattery) annotation(
      Placement(transformation(origin = {60, 0}, extent = {{-25, -15}, {25, 15}}, rotation = -90)));

  output Real SOC(unit = "1") "Pack state of charge";
  output Real SOE(unit = "1") "Pack state of energy";
  output SI.Energy E_remaining "Estimated remaining pack energy";
  output SI.Voltage v "Terminal voltage";
  output SI.Current i "Battery current, positive while discharging";
  output SI.Power P "Electrical power, positive while discharging";

protected
  Modelica.Blocks.Sources.RealExpression voltageSignal(
    y = v) "Terminal voltage published to batteryBus" annotation(
      Placement(transformation(origin = {-20, 80}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression currentSignal(
    y = i) "Terminal current published to batteryBus" annotation(
      Placement(transformation(origin = {-20, 40}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression powerSignal(
    y = P) "Terminal power published to batteryBus" annotation(
      Placement(transformation(origin = {-20, 0}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression chargeSignal(
    y = SOC) "State of charge published to batteryBus" annotation(
      Placement(transformation(origin = {-20, -40}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression energySignal(
    y = SOE) "State of energy published to batteryBus" annotation(
      Placement(transformation(origin = {-20, -80}, extent = {{10, -10}, {-10, 10}})));

  BobLib.EnergyStorage.Interfaces.BatteryBus batteryBus
    "Typed batteryBus signal contract" annotation(
      Placement(transformation(origin = {-70, -60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(batteryBus, controlBus.batteryBus) annotation(
    Line(points = {{-70, -60}, {-100, -60}}, color = {255, 204, 51}, thickness = 0.5));
  SOC = battery.SOC;
  SOE = battery.SOE;
  E_remaining = battery.E_remaining;
  v = battery.v;
  i = battery.i;
  P = battery.P;

  connect(voltageSignal.y, batteryBus.voltage) annotation(
    Line(points = {{-31, 80}, {-70, 80}, {-70, -60}}, color = {0, 0, 127}));
  connect(currentSignal.y, batteryBus.current) annotation(
    Line(points = {{-31, 40}, {-70, 40}, {-70, -60}}, color = {0, 0, 127}));
  connect(powerSignal.y, batteryBus.power) annotation(
    Line(points = {{-31, 0}, {-70, 0}, {-70, -60}}, color = {0, 0, 127}));
  connect(chargeSignal.y, batteryBus.soc) annotation(
    Line(points = {{-31, -40}, {-70, -40}, {-70, -60}}, color = {0, 0, 127}));
  connect(energySignal.y, batteryBus.soe) annotation(
    Line(points = {{-31, -80}, {-70, -80}, {-70, -60}}, color = {0, 0, 127}));

  connect(pin_p, battery.p) annotation(
    Line(points = {{100, 60}, {60, 60}, {60, 25}}, color = {0, 0, 255}));
  connect(pin_n, battery.n) annotation(
    Line(points = {{100, -60}, {60, -60}, {60, -25}}, color = {0, 0, 255}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces-facing battery pack backed by the internal Thevenin pack implementation.
It inherits <code>BobLib.EnergyStorage.Interfaces.Base</code> and
connects the standard <code>pin_p</code>/<code>pin_n</code> terminals directly
to the BobLib battery terminals. The pack publishes terminal voltage, current,
power, SOC, and SOE measurements on <code>controlBus.batteryBus</code>.
</p>
</html>"));
end BatteryPack;
