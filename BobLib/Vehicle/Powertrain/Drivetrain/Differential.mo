within BobLib.Vehicle.Powertrain.Drivetrain;

model Differential "Parameterized limited-slip differential with regularized bounded clutch lockup"
  import Modelica.SIunits;
  import Modelica.Math.exp;
  import Modelica.Math.tanh;

  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_in
    "Input from chain/gearbox" annotation(
      Placement(transformation(origin={-100,0}, extent={{-10,-10},{10,10}}), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_left
    "Left halfshaft output" annotation(
      Placement(transformation(origin={100,40}, extent={{-10,-10},{10,10}}), iconTransformation(origin = {100, 38}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_right
    "Right halfshaft output" annotation(
      Placement(transformation(origin={100,-40}, extent={{-10,-10},{10,10}}), iconTransformation(origin = {100, -38}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame
    "Differential case reaction frame" annotation(
      Placement(transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
        iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(
    Placement(transformation(origin = {0, 66}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  // Tunables. The ideal open-diff kinematic relation has no final-drive ratio;
  // keep the ratio in an upstream gear component.
  parameter Boolean use_lsd = true
    "Enable limited-slip locking torque; false leaves the ideal open-differential kinematics only"
    annotation(Evaluate = false);
  parameter Real driveSideTorqueSign = 1
    "Sign of T_in corresponding to drive-side ramp loading";
  parameter SIunits.Torque T_preload(min = 0) = 20
    "Static preload breakaway torque";
  parameter Real lockFractionAccel(unit = "1", min = 0, max = 1) = 0.35
    "Drive-side locking value S = |T_left - T_right|/|T_in|";
  parameter Real lockFractionDecel(unit = "1", min = 0, max = 1) = 0.15
    "Coast-side locking value S = |T_left - T_right|/|T_in|";
  parameter SIunits.Torque T_capacity_max(min = 0) = 1000
    "Numerical cap on static clutch lock capacity";
  parameter SIunits.Length clutchEffectiveRadius(min = 1e-6) = 1.0
    "Effective clutch torque arm used to map normal force to lock torque";
  parameter Real kineticFrictionRatio(unit = "1", min = 1e-6, max = 1) = 0.85
    "Sliding clutch capacity divided by static breakaway capacity";
  parameter SIunits.AngularVelocity w_transition(min = 1e-6) = 1.0
    "Slip speed used by the regularized plate-friction transition";
  parameter SIunits.RotationalDampingConstant c_viscous(min = 0) = 0.05
    "Small viscous slip damping included in the capped lock torque";

  // Diagnostics
  SIunits.AngularVelocity w_in;
  SIunits.AngularVelocity w_l;
  SIunits.AngularVelocity w_r;
  SIunits.AngularVelocity dw;
  SIunits.Torque T_in;
  SIunits.Torque T_open;
  Real lockFraction(unit = "1");
  SIunits.Torque T_lock_capacity_raw;
  SIunits.Torque T_lock_capacity;
  SIunits.Torque T_lock_kinetic_capacity;
  SIunits.Torque T_lock_friction_capacity;
  SIunits.Torque T_lock_viscous;
  SIunits.Torque T_lock;
  SIunits.Torque T_left_drive;
  SIunits.Torque T_right_drive;
  SIunits.Force clutchNormalForceEquivalent
    "Equivalent clutch normal force implied by capacity and effective radius";
  Real lockingValue(unit = "1") "Actual locking value |T_left - T_right|/|T_in|";
  Real torqueBiasRatio(unit = "1") "Approximate high/low output torque ratio";
  Boolean locked "True when the regularized clutch is inside the near-stick slip band";

protected
  Modelica.Mechanics.Rotational.Interfaces.Flange_a support
    "Internal 1D support tied to mountFrame through Mounting1D";

equation
  assert(T_capacity_max >= 0, "Differential: T_capacity_max must be non-negative");

  w_in = der(shaft_in.phi - support.phi);
  w_l  = der(shaft_left.phi - support.phi);
  w_r  = der(shaft_right.phi - support.phi);
  dw   = w_l - w_r;

  // Ideal open-diff kinematics.
  shaft_in.phi - support.phi =
    ((shaft_left.phi - support.phi) + (shaft_right.phi - support.phi))/2;

  // Input torque sign follows the local flange convention. Positive T_in
  // delivers positive drive torque to both halfshafts in the open-diff limit.
  T_in = shaft_in.tau;
  T_open = T_in/2;

  lockFraction =
    if noEvent(driveSideTorqueSign*T_in >= 0) then
      lockFractionAccel
    else
      lockFractionDecel;

  // For a plate/ramp LSD, the locking value S limits the output torque
  // difference: |T_left - T_right| <= S*|T_in|. Since T_lock is half of
  // that difference, the ramp contribution to lock capacity is 0.5*S*|T_in|.
  T_lock_capacity_raw =
    if use_lsd then
      T_preload + 0.5*lockFraction*abs(T_in)
    else
      0;
  T_lock_capacity = noEvent(min(T_lock_capacity_raw, T_capacity_max));
  T_lock_kinetic_capacity = kineticFrictionRatio*T_lock_capacity;
  clutchNormalForceEquivalent = T_lock_capacity/clutchEffectiveRadius;
  locked = use_lsd and noEvent(abs(dw) <= w_transition);

  // Regularized plate friction: static capacity near zero slip blends down to
  // kinetic capacity as relative wheel speed grows, then remains torque-capped.
  T_lock_friction_capacity =
    if use_lsd then
      T_lock_kinetic_capacity +
        (T_lock_capacity - T_lock_kinetic_capacity)*
        exp(-(dw/w_transition)*(dw/w_transition))
    else
      0;

  T_lock_viscous =
    if use_lsd then
      c_viscous*(-dw)
    else
      0;
  T_lock =
    if use_lsd then
      noEvent(max(
        min(T_lock_friction_capacity*tanh(-dw/w_transition) + T_lock_viscous, T_lock_capacity),
        -T_lock_capacity))
    else
      0;

  T_left_drive = T_open + T_lock;
  T_right_drive = T_open - T_lock;

  shaft_left.tau = -T_left_drive;
  shaft_right.tau = -T_right_drive;

  lockingValue = noEvent(
    abs(T_left_drive - T_right_drive)/
    max(abs(T_in), 1e-6));
  torqueBiasRatio = noEvent(
    (max(abs(T_left_drive), abs(T_right_drive)) + 1e-6)/
    (min(abs(T_left_drive), abs(T_right_drive)) + 1e-6));

  connect(mountFrame, mounting1D.frame_a) annotation(
    Line(points = {{0, 100}, {0, 76}}, color = {95, 95, 95}));
  connect(support, mounting1D.flange_b) annotation(
    Line(points = {{0, 0}, {0, 56}}));
annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {
      Ellipse(extent = {{-42, 42}, {42, -42}}, lineColor = {45, 45, 45}, fillColor = {238, 238, 238}, fillPattern = FillPattern.Solid),
      Ellipse(extent = {{-16, 16}, {16, -16}}, lineColor = {45, 45, 45}, fillColor = {210, 215, 220}, fillPattern = FillPattern.Solid),
      Line(points = {{-100, 0}, {-42, 0}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{16, 16}, {100, 38}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{16, -16}, {100, -38}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{0, 100}, {0, 42}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{-28, 0}, {28, 0}}, color = {45, 45, 45}, thickness = 1),
      Line(points = {{0, -28}, {0, 28}}, color = {45, 45, 45}, thickness = 1),
      Text(extent = {{-70, 78}, {70, 50}}, textString = "%name", lineColor = {32, 32, 32})}));
end Differential;
