within BobLibTest.PowerElectronics.Internal;

model TestInverterDCCore "Voltage protection against resistive DC sources at both bus limits"

  import Modelica.Units.SI;

  BobLib.PowerElectronics.Internal.InverterDCCore fullPack(pInverter(V_dc_max = 588)) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Sources.ConstantVoltage fullVoltage(V = 588) annotation(
    Placement(transformation(origin = {-140, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Electrical.Analog.Basic.Resistor fullResistance(R = 0.25) annotation(
    Placement(transformation(origin = {-80, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Basic.Ground fullGround annotation(
    Placement(transformation(origin = {-140, -40}, extent = {{-10, -10}, {10, 10}})));

  BobLib.PowerElectronics.Internal.InverterDCCore lowPack annotation(
    Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Sources.ConstantVoltage lowVoltage(V = 201) annotation(
    Placement(transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));

  Modelica.Electrical.Analog.Basic.Resistor lowResistance(R = 0.25) annotation(
    Placement(transformation(origin = {80, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Electrical.Analog.Basic.Ground lowGround annotation(
    Placement(transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}})));

  output SI.Voltage fullBusVoltage = fullPack.V_dc "DC voltage at the full-pack inverter";
  output SI.Voltage lowBusVoltage = lowPack.V_dc "DC voltage at the low-pack inverter";
  output SI.Power regenPower = fullPack.P_out "Regenerative input to the full-pack inverter, negative";
  output SI.Power drivePower = lowPack.P_out "Motoring output of the low-pack inverter";

equation
  fullPack.P_req = if time < 0.01 then 100 else -100;
  lowPack.P_req = if time < 0.01 then 0 else 10000;
  connect(fullVoltage.p, fullResistance.p) annotation(Line(points = {{-140, 50}, {-90, 50}}, color = {0, 0, 255}));
  connect(fullResistance.n, fullPack.p) annotation(Line(points = {{-70, 50}, {-60, 50}, {-60, 20}, {-90, 20}, {-90, 0}}, color = {0, 0, 255}));
  connect(fullVoltage.n, fullPack.n) annotation(Line(points = {{-140, 30}, {-110, 30}, {-110, -20}, {-70, -20}, {-70, 0}}, color = {0, 0, 255}));
  connect(fullVoltage.n, fullGround.p) annotation(Line(points = {{-140, 30}, {-140, -30}}, color = {0, 0, 255}));
  connect(lowVoltage.p, lowResistance.p) annotation(Line(points = {{20, 50}, {70, 50}}, color = {0, 0, 255}));
  connect(lowResistance.n, lowPack.p) annotation(Line(points = {{90, 50}, {100, 50}, {100, 20}, {70, 20}, {70, 0}}, color = {0, 0, 255}));
  connect(lowVoltage.n, lowPack.n) annotation(Line(points = {{20, 30}, {50, 30}, {50, -20}, {90, -20}, {90, 0}}, color = {0, 0, 255}));
  connect(lowVoltage.n, lowGround.p) annotation(Line(points = {{20, 30}, {20, -30}}, color = {0, 0, 255}));

  assert(fullBusVoltage <= fullPack.pInverter.V_dc_max + 1e-5,
    "Full-pack regeneration must not drive the DC bus above the voltage limit");
  assert(lowBusVoltage >= lowPack.pInverter.V_dc_min - 1e-5,
    "Low-pack motoring must not pull the DC bus below the voltage limit");
  assert(fullPack.P_loss >= -1e-5 and lowPack.P_loss >= -1e-5,
    "Voltage protection must preserve nonnegative inverter losses");

  when terminal() then
    assert(regenPower < 0 and regenPower > -100 and abs(fullPack.I_dc) < 1e-5,
      "A full pack must curtail charging while allowing regen to supply standby loss");
    assert(drivePower > 0 and drivePower < 10000,
      "A nearly depleted source must curtail motoring rather than toggle its load");
  end when;
  annotation(experiment(StopTime = 0.05, Tolerance = 1e-7),
    Diagram(coordinateSystem(extent = {{-160, -60}, {120, 80}})),
    Documentation(info = "<html><p>Regression for algebraic voltage-cutoff oscillation exposed by the car-only FMU. Both source/resistor circuits have a valid curtailed-power solution. Binary cutouts instead alternately enable and disable their own load, failing the nonlinear solve. The ordinary 500 V behavior is covered independently by TestInverterDC.</p></html>"));
end TestInverterDCCore;
