within BobLibTest.TestVehicle.TestChassis.TestSuspension.TestLinkages;

model TestTabularDamper "Asymmetric nonlinear force, sign, power and extrapolation regression"
  BobLib.Chassis.Suspension.Linkages.TabularDamper damper(
    damperTable = [-0.25, -300; -0.1, -200; 0, 0; 0.1, 100; 0.25, 150]);
  Modelica.Mechanics.Translational.Components.Fixed fixed;
  Modelica.Mechanics.Translational.Sources.Position position(exact = true);
  Modelica.Blocks.Sources.Sine motion(amplitude = 0.06, f = 1);
  Modelica.Units.SI.Velocity u;
  Modelica.Units.SI.Force expectedForce;
  Modelica.Units.SI.Power portPower;
equation
  connect(fixed.flange, damper.flange_a);
  connect(position.flange, damper.flange_b);
  connect(motion.y, position.s_ref);
  u = -0.06*2*Modelica.Constants.pi*cos(2*Modelica.Constants.pi*time);
  expectedForce = if u < -0.1 then -200 + (u + 0.1)*100/0.15
    elseif u < 0 then 2000*u
    elseif u < 0.1 then 1000*u
    else 100 + (u - 0.1)*50/0.15;
  portPower = damper.flange_a.f*der(damper.flange_a.s)
    + damper.flange_b.f*der(damper.flange_b.s);
  // Check converged event states rather than trial evaluations while the table
  // and the independent piecewise reference are selecting their next segments.
  when sample(0, 0.001) then
    assert(abs(damper.force.f - expectedForce) < 1e-6, "Incorrect nonlinear table force");
    assert(portPower >= -1e-7, "Damper creates energy");
    assert(abs(portPower - damper.dissipatedPower) < 1e-6, "Power diagnostic has incorrect sign");
    assert(damper.extrapolating == (abs(u) > 0.25), "Incorrect extrapolation flag");
  end when;
  annotation(experiment(StopTime = 1, Interval = 0.001, Tolerance = 1e-8));
end TestTabularDamper;
