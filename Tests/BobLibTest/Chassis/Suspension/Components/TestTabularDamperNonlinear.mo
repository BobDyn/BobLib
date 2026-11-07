within BobLibTest.Chassis.Suspension.Components;

model TestTabularDamperNonlinear "Asymmetric force, passivity and terminal extrapolation regression"

  import Modelica.Units.SI;

  BobLib.Chassis.Suspension.Components.TabularDamper damper(
    damperTable = [-0.25, -300; -0.1, -200; 0, 0; 0.1, 100; 0.25, 150]) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Sources.Position position(exact = true) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Blocks.Sources.Sine motion(amplitude = 0.06, f = 1) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{-10, -10}, {10, 10}})));
  SI.Velocity u "Independent compression-positive reference velocity";
  SI.Force expectedForce "Independent piecewise force including both terminal segments";
  SI.Power portPower "Power entering the two mechanical ports";

equation
  connect(fixed.flange, damper.flange_a) annotation(Line(points = {{-70, 0}, {-10, 0}}, color = {0, 127, 0}));
  connect(position.flange, damper.flange_b) annotation(Line(points = {{60, 0}, {10, 0}}, color = {0, 127, 0}));
  connect(motion.y, position.s_ref) annotation(Line(points = {{81, -50}, {95, -50}, {95, 0}, {82, 0}}, color = {0, 0, 127}));
  u = -0.06*2*Modelica.Constants.pi*cos(2*Modelica.Constants.pi*time);
  expectedForce = if u < -0.1 then -200 + (u + 0.1)*100/0.15
    elseif u < 0 then 2000*u
    elseif u < 0.1 then 1000*u
    else 100 + (u - 0.1)*50/0.15;
  portPower = damper.flange_a.f*der(damper.flange_a.s) + damper.flange_b.f*der(damper.flange_b.s);

  // Sample converged event states instead of solver trial segment selections.
  when sample(0, 0.001) then
    assert(abs(damper.force.f - expectedForce) < 1e-6, "Incorrect nonlinear damper force");
    assert(portPower >= -1e-7, "Damper creates energy");
    assert(abs(portPower - damper.dissipatedPower) < 1e-6, "Incorrect dissipated power sign");
    assert(damper.extrapolating == (abs(u) > 0.25), "Incorrect extrapolation flag");
  end when;
  annotation(experiment(StopTime = 1, Interval = 0.001, Tolerance = 1e-8),
    Diagram(coordinateSystem(extent = {{-100, -80}, {100, 40}}, preserveAspectRatio = true)));
end TestTabularDamperNonlinear;
