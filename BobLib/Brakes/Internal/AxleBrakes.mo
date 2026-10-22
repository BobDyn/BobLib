within BobLib.Brakes.Internal;
model AxleBrakes "Left/right brake rotors with explicit caliper mounting frames"

  parameter Modelica.Units.SI.AngularVelocity wRegularization(min = 0) = 0.1 "Speed band over which brake torque reverses smoothly";
  parameter Boolean usingMultiBodyChassis = false "Expose the two spatial caliper mounts" annotation(choices(checkBox = true), Evaluate = true);
  input Modelica.Units.SI.Torque leftTorqueRequest "Left brake torque magnitude";
  input Modelica.Units.SI.Torque rightTorqueRequest "Right brake torque magnitude";
  Modelica.Mechanics.Rotational.Interfaces.Flange_b leftRotor "Left rotating brake disc" annotation(Placement(transformation(origin = {-100, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b rightRotor "Right rotating brake disc" annotation(Placement(transformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a leftCaliperMount if usingMultiBodyChassis "Left caliper: connect to upright for outboard, housing for inboard" annotation(Placement(transformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a rightCaliperMount if usingMultiBodyChassis "Right caliper: connect to upright for outboard, housing for inboard" annotation(Placement(transformation(origin = {100, -20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Torque leftBrake(useSupport = usingMultiBodyChassis) annotation(Placement(transformation(origin = {-50, 20}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Torque rightBrake(useSupport = usingMultiBodyChassis) annotation(Placement(transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D leftCaliper(n = {0, 1, 0}) if usingMultiBodyChassis annotation(Placement(visible = usingMultiBodyChassis, transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D rightCaliper(n = {0, 1, 0}) if usingMultiBodyChassis annotation(Placement(visible = usingMultiBodyChassis, transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}})));

protected
  outer Modelica.Mechanics.MultiBody.World world;

equation
  assert(wRegularization >= 0, "AxleBrakes: wRegularization must be nonnegative");
  assert(
    not usingMultiBodyChassis or world.driveTrainMechanics3D,
    "AxleBrakes: spatial caliper mounts require world.driveTrainMechanics3D=true");
  leftBrake.tau = -leftTorqueRequest*noEvent(if der(leftRotor.phi)>wRegularization then 1 elseif der(leftRotor.phi)<-wRegularization then -1 else der(leftRotor.phi)/max(wRegularization, 1e-6));
  rightBrake.tau = -rightTorqueRequest*noEvent(if der(rightRotor.phi)>wRegularization then 1 elseif der(rightRotor.phi)<-wRegularization then -1 else der(rightRotor.phi)/max(wRegularization, 1e-6));
  connect(leftBrake.flange, leftRotor) annotation(Line(points = {{-60, 20}, {-100, 20}}));
  connect(rightBrake.flange, rightRotor) annotation(Line(points = {{60, 20}, {100, 20}}));
  connect(
    leftBrake.support,
    leftCaliper.flange_b) annotation(Line(visible = usingMultiBodyChassis, points = {{-50, 10}, {-50, 0}, {-30, 0}, {-30, -20}, {-40, -20}}));
  connect(
    rightBrake.support,
    rightCaliper.flange_b) annotation(Line(visible = usingMultiBodyChassis, points = {{50, 10}, {50, 0}, {30, 0}, {30, -20}, {40, -20}}));
  connect(
    leftCaliper.frame_a,
    leftCaliperMount) annotation(Line(visible = usingMultiBodyChassis, points = {{-50, -30}, {-50, -40}, {-80, -40}, {-80, -20}, {-100, -20}}, color = {95, 95, 95}));
  connect(
    rightCaliper.frame_a,
    rightCaliperMount) annotation(Line(visible = usingMultiBodyChassis, points = {{50, -30}, {50, -40}, {80, -40}, {80, -20}, {100, -20}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -50}, {100, 50}}), graphics = {
    Ellipse(extent = {{-65, 40}, {-25, 0}}, fillPattern = FillPattern.Solid, fillColor = {175, 175, 175}),
    Ellipse(extent = {{25, 40}, {65, 0}}, fillPattern = FillPattern.Solid, fillColor = {175, 175, 175}),
    Line(points = {{-100, 20}, {-65, 20}}), Line(points = {{65, 20}, {100, 20}}),
    Line(visible = usingMultiBodyChassis, points = {{-100, -20}, {-45, -20}, {-45, 0}}, color = {95, 95, 95}),
    Line(visible = usingMultiBodyChassis, points = {{100, -20}, {45, -20}, {45, 0}}, color = {95, 95, 95}),
    Text(extent = {{-100, 80}, {100, 55}}, textString = "%name", textColor = {0, 0, 255})}),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})),
    Documentation(info = "<html><p>One axle's two brake actuators. Rotor flanges connect to the existing wheel axes for outboard braking or the driveline brake shafts for inboard braking. Each caliper mount connects independently to its unsprung upright or sprung housing. Mounting1D resolves support torque about that frame's local y axis. The vehicle-level brake subsystem owns pedal demand, bias and bus publication. This regularized torque model has no static holding, thermal or hydraulic dynamics.</p></html>"));
end AxleBrakes;
