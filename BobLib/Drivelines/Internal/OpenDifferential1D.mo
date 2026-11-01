within BobLib.Drivelines.Internal;
model OpenDifferential1D "Ideal equal-torque differential"

  extends BobLib.Icons.DifferentialIcon;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_in
    "Input from final drive" annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_left
    "Left halfshaft output" annotation(
      Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_right
    "Right halfshaft output" annotation(
      Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

equation
  2*shaft_in.phi = shaft_left.phi + shaft_right.phi;
  shaft_left.tau = -shaft_in.tau/2;
  shaft_right.tau = shaft_left.tau;
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
end OpenDifferential1D;
