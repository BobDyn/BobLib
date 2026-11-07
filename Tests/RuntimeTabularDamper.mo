within;

model RuntimeTabularDamper "Runtime curves exercised across compression, rebound and extrapolation"

  parameter Boolean useRuntimeDamperTables = false "Load the named curve file when the simulation starts"
    annotation(Evaluate = false, choices(checkBox = true));
  parameter String damperTableFile = "damper_tables.txt" "File containing named damper force-velocity curves"
    annotation(Evaluate = false);
  parameter String damperTableName = "front" "Curve name selected from the runtime file";

  BobLib.Chassis.Suspension.Components.TabularDamper damper(
    damperTable = [-0.25, -250; 0, 0; 0.25, 250],
    useRuntimeTable = useRuntimeDamperTables,
    runtimeTableFile = damperTableFile,
    runtimeTableName = damperTableName) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Position position(exact = true) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.Sine motion(amplitude = 0.06, f = 1) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{-10, -10}, {10, 10}})));
  output Modelica.Units.SI.Velocity compressionVelocity = -damper.v_rel "Compression-positive shaft speed";
  output Modelica.Units.SI.Force damperForce = damper.force.f "Signed resisting force";

equation
  connect(fixed.flange, damper.flange_a) annotation(Line(points = {{-70, 0}, {-10, 0}}, color = {0, 127, 0}));
  connect(position.flange, damper.flange_b) annotation(Line(points = {{60, 0}, {10, 0}}, color = {0, 127, 0}));
  connect(motion.y, position.s_ref) annotation(Line(points = {{81, -50}, {95, -50}, {95, 0}, {82, 0}}, color = {0, 0, 127}));

  when sample(0, 0.001) then
    assert(damper.dissipatedPower >= -1e-7, "Runtime damper creates energy");
  end when;
  annotation(experiment(StopTime = 1, Interval = 0.001, Tolerance = 1e-8),
    Diagram(coordinateSystem(extent = {{-100, -80}, {100, 40}}, preserveAspectRatio = true)));
end RuntimeTabularDamper;
