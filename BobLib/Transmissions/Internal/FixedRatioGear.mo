within BobLib.Transmissions.Internal;
model FixedRatioGear

  "Reusable fixed-ratio rotational gear core"
  extends VehicleInterfaces.Icons.Transmission;

  parameter Boolean useSupport = false "Expose housing reaction" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Real gearRatio = 3.31
    "Input speed divided by output speed" annotation(Dialog(group = "Operating settings"));
  parameter Real efficiency(min = Modelica.Constants.small, max = 1) = 1
    "Mechanical efficiency in either power-flow direction" annotation(Evaluate = false);
  Modelica.Mechanics.Rotational.Interfaces.Support support if useSupport annotation(
    Placement(transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a inputFlange
    "Input shaft" annotation(
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b outputFlange
    "Output shaft" annotation(
      Placement(transformation(extent = {{90, -10}, {110, 10}})));

protected
  Modelica.Mechanics.Rotational.Components.LossyGear gear(
    lossTable = [0, efficiency, efficiency, 0, 0],
    ratio = gearRatio,
    useSupport = useSupport) annotation(
      Placement(transformation(extent = {{-20, -20}, {20, 20}})));

initial equation
  assert(efficiency > 0 and efficiency <= 1, "Transmission efficiency must be in (0, 1]");

equation
  connect(gear.support, support) annotation(
    Line(visible = useSupport, points = {{0, -20}, {0, -60}}, color = {0, 0, 0}));
  connect(inputFlange, gear.flange_a) annotation(
    Line(points = {{-100, 0}, {-20, 0}}));
  connect(gear.flange_b, outputFlange) annotation(
    Line(points = {{20, 0}, {100, 0}}));

  annotation(Documentation(info = "<html>
<p>
Model <code>FixedRatioGear</code> contains the reusable fixed-ratio gear
relation used by public transmission adapters. MSL LossyGear applies the same configurable efficiency in both power-flow directions, including regeneration. Zero bearing-friction terms avoid introducing a separate friction calibration. This lumped model does not predict chain tension or individual mounting loads.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}}, preserveAspectRatio = true)));
end FixedRatioGear;
