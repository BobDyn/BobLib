within BobLibVehicleInterfaces.Transmissions.Internal;
model FixedRatioGear
  "Reusable fixed-ratio rotational gear core"
  parameter Real gearRatio = 3.31
    "Input speed divided by output speed";

  Modelica.Mechanics.Rotational.Interfaces.Flange_a inputFlange
    "Input shaft" annotation(
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b outputFlange
    "Output shaft" annotation(
      Placement(transformation(extent = {{90, -10}, {110, 10}})));

protected
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(
    ratio = gearRatio,
    useSupport = false) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  connect(inputFlange, idealGear.flange_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(idealGear.flange_b, outputFlange) annotation(
    Line(points = {{10, 0}, {100, 0}}));

  annotation(Documentation(info = "<html>
<p>
Model <code>FixedRatioGear</code> contains the reusable fixed-ratio gear
relation used by public transmission adapters.
</p>
</html>"));
end FixedRatioGear;
