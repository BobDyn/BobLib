within BobLib.Vehicle.Powertrain.Battery.Templates;
partial model BatteryBase

  import SI = Modelica.Units.SI;

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
  parameter Integer Ns(min = 1) = 140 "Number of cells in series";
  parameter Integer Np(min = 1) = 4 "Number of cells in parallel";

  // Initial condition
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1 "Initial battery state of charge";

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
  SOC_state = SOC_start;

end BatteryBase;
