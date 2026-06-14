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

equation
  SOC = battery.SOC;
  SOE = battery.SOE;
  E_remaining = battery.E_remaining;
  v = battery.v;
  i = battery.i;
  P = battery.P;

  connect(pin_p, battery.p) annotation(
    Line(points = {{100, 60}, {-40, 60}, {-40, 0}}, color = {0, 0, 255}));
  connect(pin_n, battery.n) annotation(
    Line(points = {{100, -60}, {40, -60}, {40, 0}}, color = {0, 0, 255}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces-facing battery pack backed by the internal Thevenin pack implementation.
It inherits <code>VehicleInterfaces.EnergyStorage.Interfaces.Base</code> and
connects the standard <code>pin_p</code>/<code>pin_n</code> terminals directly
to the BobLib battery terminals.
</p>
</html>"));
end BatteryPack;
