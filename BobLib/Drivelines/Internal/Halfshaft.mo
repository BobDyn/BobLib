within BobLib.Drivelines.Internal;
model Halfshaft "Lumped halfshaft inertia and replaceable torsional compliance"

  parameter BobLib.Records.VehicleRecord.Drivelines.HalfshaftRecord pHalfshaft "Shared per-halfshaft inertia, torsional stiffness and damping calibration" annotation(Dialog(group = "Calibration"));
  parameter Boolean useWheelFrame = false "Include shaft carrier-motion and reaction transport" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean useInboardBrake = false "Expose the brake rotor flange at the differential-side end of the shaft compliance" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean includeRotorEffects = false
    "Include gyroscopic and carrier-acceleration effects when mounted" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean useMount = false "Resolve rotor reaction into a housing frame" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Modelica.Mechanics.MultiBody.Types.Axis shaftAxis = {0, 1, 0} "Positive shaft rotation axis resolved in the non-spinning housing frame" annotation(Dialog(group = "Operating settings"), Evaluate = true);
  parameter Modelica.Units.SI.AngularVelocity initialAngularVelocity = 0 "Initial shaft angular velocity; enforced when fixInitialSpeed is true" annotation(Dialog(group = "Initialization"));
  parameter Boolean fixInitialSpeed = false "Fix shaft speed to initialAngularVelocity during initialization" annotation(Dialog(group = "Initialization"), choices(checkBox = true), Evaluate = true);
  replaceable model ComplianceModel = Modelica.Mechanics.Rotational.Components.SpringDamper(
    c = pHalfshaft.stiffness, d = pHalfshaft.damping, stateSelect = StateSelect.never)
    constrainedby Modelica.Mechanics.Rotational.Interfaces.PartialCompliant
    "Torsional compliance law" annotation(choicesAllMatching = true);
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a "Differential side" annotation(
    Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b "Wheel side" annotation(
    Placement(transformation(extent = {{90, -10}, {110, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mount if useMount or useWheelFrame annotation(
    Placement(transformation(origin = {0, -60}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a wheelFrame if useWheelFrame annotation(
    Placement(transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a brake if useInboardBrake
    "Brake rotor at differential-side end of compliance" annotation(
    Placement(transformation(origin = {-30, 60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

  BobLib.Utilities.Mechanics.MultiBody.InboardShaft transport(
    useSpatialSupports = useMount, inboardAxis = shaftAxis) if useWheelFrame annotation(
    Placement(visible = useWheelFrame, transformation(origin = {65, -9}, extent = {{-15, -15}, {15, 15}})));

  Modelica.Mechanics.Rotational.Components.Inertia rotor(
    J = pHalfshaft.inertia, w(
      start = initialAngularVelocity,
      fixed = fixInitialSpeed)) if not (useMount and includeRotorEffects) annotation(
    Placement(
      visible = not (useMount and includeRotorEffects),
      transformation(origin = {-60, 0}, extent = {{-15, -15}, {15, 15}})));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D spatialRotor(
    J = pHalfshaft.inertia, n = shaftAxis, animation = false,
    w(start = initialAngularVelocity, fixed = fixInitialSpeed)) if useMount and includeRotorEffects annotation(
    Placement(
      visible = useMount and includeRotorEffects,
      transformation(origin = {-60, 0}, extent = {{-15, -15}, {15, 15}})));

  ComplianceModel compliance(phi_rel(start = 0)) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-15, -15}, {15, 15}})));
  output Modelica.Units.SI.Torque tau = compliance.tau "Wheel-side reaction torque (MSL compliance convention)";

initial equation
  assert(pHalfshaft.inertia > 0, "Halfshaft: inertia must be positive");
  assert(pHalfshaft.stiffness >= 0 and pHalfshaft.damping >= 0,
    "Halfshaft: stiffness and damping must be nonnegative");
  compliance.phi_rel = 0;

equation
  if useMount and not useWheelFrame and not includeRotorEffects then
    mount.f = zeros(3);
    mount.t = zeros(3);
  end if;
  connect(
    flange_a,
    rotor.flange_a) annotation(Line(visible = not (useMount and includeRotorEffects), points = {{-100, 0}, {-75, 0}}));
  connect(
    flange_a,
    spatialRotor.flange_a) annotation(Line(visible = useMount and includeRotorEffects, points = {{-100, 0}, {-75, 0}}));
  connect(
    rotor.flange_b,
    compliance.flange_a) annotation(Line(visible = not (useMount and includeRotorEffects), points = {{-45, 0}, {-15, 0}}));
  connect(
    spatialRotor.flange_b,
    compliance.flange_a) annotation(Line(visible = useMount and includeRotorEffects, points = {{-45, 0}, {-15, 0}}));
  connect(
    brake,
    compliance.flange_a) annotation(Line(visible = useInboardBrake, points = {{-30, 60}, {-30, 0}, {-15, 0}}));

  if useWheelFrame then
    connect(
      compliance.flange_b,
      transport.inboard) annotation(Line(visible = useWheelFrame, points = {{15, 0}, {50, 0}}));
    connect(
      transport.outboard,
      flange_b) annotation(Line(visible = useWheelFrame, points = {{80, 0}, {100, 0}}));
  else
    connect(compliance.flange_b, flange_b) annotation(Line(visible = not useWheelFrame, points = {{15, 0}, {100, 0}}));
  end if;
  connect(
    transport.sprungFrame,
    mount) annotation(Line(visible = useWheelFrame, points = {{50, -15}, {50, -45}, {0, -45}, {0, -60}}, color = {95, 95, 95}));
  connect(
    transport.uprightFrame,
    wheelFrame) annotation(Line(visible = useWheelFrame, points = {{80, -15}, {100, -15}, {100, -60}}, color = {95, 95, 95}));
  connect(
    spatialRotor.frame_a,
    mount) annotation(Line(visible = useMount and includeRotorEffects, points = {{-60, -15}, {-60, -45}, {0, -45}, {0, -60}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}}, preserveAspectRatio = true)),
    Icon(coordinateSystem(preserveAspectRatio = true), graphics = {
      Rectangle(extent = {{-100, 8}, {-60, -8}}, lineColor = {64, 64, 64}, fillColor = {160, 160, 160}, fillPattern = FillPattern.Solid),
      Rectangle(extent = {{-60, 32}, {-25, -32}}, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid),
      Line(points = {{-25, 0}, {-15, 0}, {-5, 25}, {10, -25}, {25, 25}, {40, -25}, {50, 0}, {100, 0}}),
      Line(
        visible = useMount or useWheelFrame,
        points = {{-42, -32}, {-42, -70}, {0, -70}, {0, -100}},
        color = {95, 95, 95}),
      Text(extent = {{-150, 85}, {150, 45}}, textString = "%name", lineColor = {0, 0, 255})}),
    Documentation(info = "<html><p>Lumped rotational inertia followed by a replaceable compliance law. Both sides of a differential receive the same pHalfshaft record. Initial twist is zero; the assembly decides which shaft speeds require initial equations.</p><p>The optional mount supplies the sprung reaction reference. includeRotorEffects selects Rotor1D carrier/gyroscopic corrections when mounted and requires world.driveTrainMechanics3D. Shaft reaction transport remains active with scalar inertia. This is a torsional model, with ideal carrier-motion and torque-reaction transport when useWheelFrame is enabled; without CV-joint geometry, plunge forces or bending. The inboard brake taps the shaft before compliance; outboard brakes attach at the external wheel hub. Vehicle mass properties already include the locked rotor inertia.</p><h4>Parameter setup</h4><p>Set physical calibration once through <code>pHalfshaft</code>. The axle-level driveline copies this per-half record to both shafts. <code>useInboardBrake</code> exposes the brake rotor port; <code>useWheelFrame</code> adds carrier motion and the upright reaction path; <code>useMount</code> supplies the housing reference. Rotor gyroscopic effects require both <code>useMount</code> and <code>includeRotorEffects</code>. These connector and physics switches select topology before translation. Set <code>initialAngularVelocity</code> with <code>fixInitialSpeed=true</code> to impose the initial speed; otherwise it is only a start guess.</p>
</html>"));
end Halfshaft;
