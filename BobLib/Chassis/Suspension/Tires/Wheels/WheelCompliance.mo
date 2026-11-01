within BobLib.Chassis.Suspension.Tires.Wheels;
model WheelCompliance "Massless six-axis elastic wheel-stack compliance"

  extends PartialWheelCompliance;

  Modelica.Mechanics.MultiBody.Joints.Prismatic slideX(
    n = {1, 0, 0},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {-150, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Components.Spring slideXLaw(c = pCompliance.translationStiffness[1]) annotation(Placement(transformation(origin = {-150, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Prismatic slideY(
    n = {0, 1, 0},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Components.Spring slideYLaw(c = pCompliance.translationStiffness[2]) annotation(Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Prismatic slideZ(
    n = {0, 0, 1},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Components.Spring slideZLaw(c = pCompliance.translationStiffness[3]) annotation(Placement(transformation(origin = {-30, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute rotateX(
    n = {1, 0, 0},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Spring rotateXLaw(c = pCompliance.rotationStiffness[1]) annotation(Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute rotateY(
    n = {0, 1, 0},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Spring rotateYLaw(c = pCompliance.rotationStiffness[2]) annotation(Placement(transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Joints.Revolute rotateZ(
    n = {0, 0, 1},
    useAxisFlange = true,
    animation = false) annotation(Placement(transformation(origin = {150, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Spring rotateZLaw(c = pCompliance.rotationStiffness[3]) annotation(Placement(transformation(origin = {150, 40}, extent = {{-10, -10}, {10, 10}})));

equation
  assert(
    min(pCompliance.translationStiffness)>0 and min(pCompliance.rotationStiffness)>0,
    "Wheel compliance requires positive stiffness on all six axes");
  assert(
    pCompliance.referenceMass>0 and min(pCompliance.referenceInertia)>0 and pCompliance.dampingRatio>=0,
    "Wheel compliance damping references must be positive and damping ratio nonnegative");
  connect(frame_a, slideX.frame_a) annotation(Line(points = {{-200, 0}, {-160, 0}}, color = {95, 95, 95}));
  connect(slideX.support, slideXLaw.flange_a) annotation(Line(points = {{-154, 6}, {-154, 20}, {-166, 20}, {-166, 40}, {-160, 40}}, color = {0, 127, 0}));
  connect(slideX.axis, slideXLaw.flange_b) annotation(Line(points = {{-142, 6}, {-142, 20}, {-134, 20}, {-134, 40}, {-140, 40}}, color = {0, 127, 0}));
  connect(slideX.frame_b, slideY.frame_a) annotation(Line(points = {{-140, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(slideY.support, slideYLaw.flange_a) annotation(Line(points = {{-94, 6}, {-94, 20}, {-106, 20}, {-106, 40}, {-100, 40}}, color = {0, 127, 0}));
  connect(slideY.axis, slideYLaw.flange_b) annotation(Line(points = {{-82, 6}, {-82, 20}, {-74, 20}, {-74, 40}, {-80, 40}}, color = {0, 127, 0}));
  connect(slideY.frame_b, slideZ.frame_a) annotation(Line(points = {{-80, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(slideZ.support, slideZLaw.flange_a) annotation(Line(points = {{-34, 6}, {-34, 20}, {-46, 20}, {-46, 40}, {-40, 40}}, color = {0, 127, 0}));
  connect(slideZ.axis, slideZLaw.flange_b) annotation(Line(points = {{-22, 6}, {-22, 20}, {-14, 20}, {-14, 40}, {-20, 40}}, color = {0, 127, 0}));
  connect(slideZ.frame_b, rotateX.frame_a) annotation(Line(points = {{-20, 0}, {20, 0}}, color = {95, 95, 95}));
  connect(rotateX.support, rotateXLaw.flange_a) annotation(Line(points = {{24, 10}, {24, 20}, {14, 20}, {14, 40}, {20, 40}}, color = {0, 0, 0}));
  connect(rotateX.axis, rotateXLaw.flange_b) annotation(Line(points = {{30, 10}, {30, 20}, {46, 20}, {46, 40}, {40, 40}}, color = {0, 0, 0}));
  connect(rotateX.frame_b, rotateY.frame_a) annotation(Line(points = {{40, 0}, {80, 0}}, color = {95, 95, 95}));
  connect(rotateY.support, rotateYLaw.flange_a) annotation(Line(points = {{84, 10}, {84, 20}, {74, 20}, {74, 40}, {80, 40}}, color = {0, 0, 0}));
  connect(rotateY.axis, rotateYLaw.flange_b) annotation(Line(points = {{90, 10}, {90, 20}, {106, 20}, {106, 40}, {100, 40}}, color = {0, 0, 0}));
  connect(rotateY.frame_b, rotateZ.frame_a) annotation(Line(points = {{100, 0}, {140, 0}}, color = {95, 95, 95}));
  connect(rotateZ.support, rotateZLaw.flange_a) annotation(Line(points = {{144, 10}, {144, 20}, {134, 20}, {134, 40}, {140, 40}}, color = {0, 0, 0}));
  connect(rotateZ.axis, rotateZLaw.flange_b) annotation(Line(points = {{150, 10}, {150, 20}, {166, 20}, {166, 40}, {160, 40}}, color = {0, 0, 0}));
  connect(rotateZ.frame_b, frame_b) annotation(Line(points = {{160, 0}, {200, 0}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -40}, {200, 120}})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{-100, 0}, {-60, 0}, {-40, 30}, {-20, -30}, {0, 30}, {20, -30}, {40, 30}, {60, 0}, {100, 0}}), Text(extent = {{-150, 80}, {150, 50}}, textString = "%name", textColor = {0, 0, 255})}), Documentation(info = "<html><p>Three local translations followed by an x-y-z Cardan rotation chain. Each coordinate has a spring. The derived DampedWheelCompliance adds parallel dampers. MSL joints transmit the complete force/moment wrench, including translation lever arms. Rotational rates are about successive axes, coinciding with the aligned wheel axes at zero deflection. This lumped model is intended for small elastic wheel-stack deflections, not free articulation.</p><p>No mass or inertia is added. Effective mass/inertia in the record derive damping only. The upstream frame retains the caliper support. Rigs select this elastic-only variant; removing other inertial and damping effects from the enclosing assembly is also necessary for a state-free equilibrium solve.</p></html>"));
end WheelCompliance;
