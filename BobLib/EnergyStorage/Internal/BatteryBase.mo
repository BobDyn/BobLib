within BobLib.EnergyStorage.Internal;
partial model BatteryBase

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.EnergyStorage.BatteryPackRecord pBattery "Cell topology, electrical maps and initial charge" annotation(Dialog(group = "Calibration"));

  // Electrical terminals
  Modelica.Electrical.Analog.Interfaces.PositivePin p "Positive DC terminal" annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin n "Negative DC terminal" annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  // Cell topology (structural parameters)

  // Initial condition

  // Physical state (NOT output)
  Real SOC(unit = "1", min = 0, max = 1) "Pack state of charge";
  Real SOC_state(unit = "1") "Integrator state for SOC, clamped by SOC for diagnostics";
  Real SOE(unit = "1", min = 0, max = 1) "Pack state of energy";

  // Internal electrical quantities (NOT outputs)
  SI.Voltage v "Terminal voltage (p.v - n.v)";
  SI.Current i "Battery current (positive -> discharge)";
  SI.Power   P "Electrical power (i * v)";
  SI.Energy E_remaining "Estimated remaining pack energy";

equation

  // Kirchhoff-consistent relations
  v = p.v - n.v;
  i = n.i;
  P = i * v;
  SOC = noEvent(min(max(SOC_state, 0), 1));

initial equation
  SOC_state = pBattery.SOC_start;

  annotation(
    Documentation(info = "<html>
<p>
Partial model <code>BatteryBase</code> defines common battery-pack parameters, electrical pins, and output quantities.
</p>
<p>
Concrete battery models extend this base to provide voltage, current, state-of-charge, and loss behavior for the energy-storage adapter.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)));
end BatteryBase;
