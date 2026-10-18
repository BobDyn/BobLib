within;

model RuntimeTabularDamper "Runtime file lookup exercised across compression, rebound and extrapolation"

  parameter Boolean useRuntimeDamperTables = false annotation(Evaluate = false);
  parameter String damperTableFile = "damper_tables.txt" annotation(Evaluate = false);

  BobLib.Chassis.Suspension.Linkages.TabularDamper damper(
    damperTable = [-0.25, -250; 0, 0; 0.25, 250],
    useRuntimeTable = useRuntimeDamperTables,
    runtimeTableFile = damperTableFile,
    runtimeTableName = "front");
  Modelica.Mechanics.Translational.Components.Fixed fixed;

  Modelica.Mechanics.Translational.Sources.Position position(exact = true);

  Modelica.Blocks.Sources.Sine motion(amplitude = 0.06, f = 1);
  output Modelica.Units.SI.Velocity compressionVelocity = -damper.v_rel;
  output Modelica.Units.SI.Force damperForce = damper.force.f;

equation
  connect(fixed.flange, damper.flange_a);
  connect(position.flange, damper.flange_b);
  connect(motion.y, position.s_ref);

  when sample(0, 0.001) then
    assert(damper.dissipatedPower >= -1e-7, "Runtime damper creates energy");
  end when;
  annotation(experiment(StopTime = 1, Interval = 0.001, Tolerance = 1e-8));
end RuntimeTabularDamper;
