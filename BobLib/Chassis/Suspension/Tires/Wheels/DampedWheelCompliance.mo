within BobLib.Chassis.Suspension.Tires.Wheels;

model DampedWheelCompliance "Six-axis wheel-stack elasticity with derived parallel damping"

  extends WheelCompliance;

  Modelica.Mechanics.Translational.Components.Damper slideXDamping(d = pCompliance.translationDamping[1]) annotation(Placement(transformation(origin = {-150, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Components.Damper slideYDamping(d = pCompliance.translationDamping[2]) annotation(Placement(transformation(origin = {-90, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Components.Damper slideZDamping(d = pCompliance.translationDamping[3]) annotation(Placement(transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Damper rotateXDamping(d = pCompliance.rotationDamping[1]) annotation(Placement(transformation(origin = {30, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Damper rotateYDamping(d = pCompliance.rotationDamping[2]) annotation(Placement(transformation(origin = {90, 80}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Damper rotateZDamping(d = pCompliance.rotationDamping[3]) annotation(Placement(transformation(origin = {150, 80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(
    slideXDamping.flange_a,
    slideXLaw.flange_a) annotation(Line(points = {{-160, 80}, {-166, 80}, {-166, 40}, {-160, 40}}, color = {0, 127, 0}));
  connect(
    slideXDamping.flange_b,
    slideXLaw.flange_b) annotation(Line(points = {{-140, 80}, {-134, 80}, {-134, 40}, {-140, 40}}, color = {0, 127, 0}));
  connect(
    slideYDamping.flange_a,
    slideYLaw.flange_a) annotation(Line(points = {{-100, 80}, {-106, 80}, {-106, 40}, {-100, 40}}, color = {0, 127, 0}));
  connect(
    slideYDamping.flange_b,
    slideYLaw.flange_b) annotation(Line(points = {{-80, 80}, {-74, 80}, {-74, 40}, {-80, 40}}, color = {0, 127, 0}));
  connect(
    slideZDamping.flange_a,
    slideZLaw.flange_a) annotation(Line(points = {{-40, 80}, {-46, 80}, {-46, 40}, {-40, 40}}, color = {0, 127, 0}));
  connect(
    slideZDamping.flange_b,
    slideZLaw.flange_b) annotation(Line(points = {{-20, 80}, {-14, 80}, {-14, 40}, {-20, 40}}, color = {0, 127, 0}));
  connect(
    rotateXDamping.flange_a,
    rotateXLaw.flange_a) annotation(Line(points = {{20, 80}, {14, 80}, {14, 40}, {20, 40}}, color = {0, 0, 0}));
  connect(
    rotateXDamping.flange_b,
    rotateXLaw.flange_b) annotation(Line(points = {{40, 80}, {46, 80}, {46, 40}, {40, 40}}, color = {0, 0, 0}));
  connect(
    rotateYDamping.flange_a,
    rotateYLaw.flange_a) annotation(Line(points = {{80, 80}, {74, 80}, {74, 40}, {80, 40}}, color = {0, 0, 0}));
  connect(
    rotateYDamping.flange_b,
    rotateYLaw.flange_b) annotation(Line(points = {{100, 80}, {106, 80}, {106, 40}, {100, 40}}, color = {0, 0, 0}));
  connect(
    rotateZDamping.flange_a,
    rotateZLaw.flange_a) annotation(Line(points = {{140, 80}, {134, 80}, {134, 40}, {140, 40}}, color = {0, 0, 0}));
  connect(
    rotateZDamping.flange_b,
    rotateZLaw.flange_b) annotation(Line(points = {{160, 80}, {166, 80}, {166, 40}, {160, 40}}, color = {0, 0, 0}));
  annotation(Documentation(info = "<html><p>Adds a parallel viscous damper to each elastic coordinate. Damping is derived from the calibration reference mass/inertia and damping ratio; no physical mass is added. Use WheelCompliance for equilibrium rigs. This variant can introduce relaxation states even with no attached mass.</p></html>"));
end DampedWheelCompliance;
