within BobLib.PowerElectronics.Interfaces;

partial model Base "DC power-electronics ports and vehicle control bus"

  extends BobLib.Icons.InverterDCIcon;

  Modelica.Electrical.Analog.Interfaces.PositivePin p
    "DC bus positive" annotation(
      Placement(transformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 62}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin n
    "DC bus negative" annotation(
      Placement(transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.PositivePin motor_p
    "Motor-side positive pin" annotation(
      Placement(transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin motor_n
    "Motor-side negative pin" annotation(
      Placement(transformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Interfaces.ControlBus controlBus
    "VehicleInterfaces control bus carrying electric-drive controller commands" annotation(
      Placement(transformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}),
        iconTransformation(origin = {-60, 100}, extent = {{-20, -20}, {20, 20}})));

  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
end Base;
