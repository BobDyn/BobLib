within BobLib.Vehicle.Powertrain;

model PTNPlaceholder
  Modelica.Mechanics.Rotational.Sources.Torque2 leftTorque annotation(
    Placement(transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Sources.Torque2 rightTorque annotation(
    Placement(transformation(origin = {30, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(
    Placement(transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame annotation(
    Placement(transformation(origin = {0, 40}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 20}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b leftFlange annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b rightFlange annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput u annotation(
    Placement(transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {0, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Math.Gain gain(k = 1/2)  annotation(
    Placement(transformation(origin = {0, -25}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));

equation
  connect(mountFrame, mounting1D.frame_a) annotation(
    Line(points = {{0, 40}, {0, 0}}));
  connect(mounting1D.flange_b, leftTorque.flange_a) annotation(
    Line(points = {{-10, -10}, {-20, -10}}));
  connect(leftTorque.flange_b, leftFlange) annotation(
    Line(points = {{-40, -10}, {-50, -10}, {-50, 0}, {-60, 0}}));
  connect(rightTorque.flange_a, mounting1D.flange_b) annotation(
    Line(points = {{20, -10}, {-10, -10}}));
  connect(rightTorque.flange_b, rightFlange) annotation(
    Line(points = {{40, -10}, {50, -10}, {50, 0}, {60, 0}}));
  connect(u, gain.u) annotation(
    Line(points = {{0, -50}, {0, -30}}, color = {0, 0, 127}));
  connect(gain.y, leftTorque.tau) annotation(
    Line(points = {{0, -20}, {-30, -20}, {-30, -14}}, color = {0, 0, 127}));
  connect(gain.y, rightTorque.tau) annotation(
    Line(points = {{0, -20}, {30, -20}, {30, -14}}, color = {0, 0, 127}));

annotation(
    Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}})),
  Icon(coordinateSystem(extent = {{-100, -20}, {100, 20}}), graphics = {Line(points = {{0, -20}, {0, 20}}), Line(points = {{-100, 0}, {100, 0}})}));
end PTNPlaceholder;
