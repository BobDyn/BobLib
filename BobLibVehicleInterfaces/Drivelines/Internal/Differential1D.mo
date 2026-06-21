within BobLibVehicleInterfaces.Drivelines.Internal;
model Differential1D

  "1D limited-slip differential using BobLib differential equations"
  import SI = Modelica.Units.SI;
  import Modelica.Math.exp;
  import Modelica.Math.tanh;

  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_in
    "Input from final drive" annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_left
    "Left halfshaft output" annotation(
      Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_right
    "Right halfshaft output" annotation(
      Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  parameter Boolean use_lsd = true
    "Enable limited-slip locking torque";
  parameter Real driveSideTorqueSign = 1
    "Sign of T_in corresponding to drive-side ramp loading";
  parameter SI.Torque T_preload(min = 0) = 20
    "Differential LSD static preload breakaway torque";
  parameter Real lockFractionAccel(unit = "1", min = 0, max = 1) = 0.35
    "Differential drive-side locking value";
  parameter Real lockFractionDecel(unit = "1", min = 0, max = 1) = 0.15
    "Differential coast-side locking value";
  parameter SI.Torque T_capacity_max(min = 0) = 1000
    "Differential maximum static lock capacity";
  parameter SI.Length clutchEffectiveRadius(min = 1e-6) = 1.0
    "Differential LSD effective clutch torque arm";
  parameter Real kineticFrictionRatio(unit = "1", min = 1e-6, max = 1) = 0.85
    "Differential sliding clutch capacity divided by static capacity";
  parameter SI.AngularVelocity w_transition(min = 1e-6) = 1.0
    "Differential LSD regularized clutch slip transition speed";
  parameter SI.RotationalDampingConstant c_viscous(min = 0) = 0.05
    "Differential small viscous slip damping included in capped lock torque";

  SI.AngularVelocity w_in;
  SI.AngularVelocity w_l;
  SI.AngularVelocity w_r;
  SI.AngularVelocity dw;
  SI.Torque T_in;
  SI.Torque T_open;
  Real lockFraction(unit = "1");
  SI.Torque T_lock_capacity_raw;
  SI.Torque T_lock_capacity;
  SI.Torque T_lock_kinetic_capacity;
  SI.Torque T_lock_friction_capacity;
  SI.Torque T_lock_viscous;
  SI.Torque T_lock;
  SI.Torque T_left_drive;
  SI.Torque T_right_drive;
  SI.Force clutchNormalForceEquivalent
    "Equivalent clutch normal force implied by capacity and effective radius";
  Real lockingValue(unit = "1") "Actual locking value |T_left - T_right|/|T_in|";
  Real torqueBiasRatio(unit = "1") "Approximate high/low output torque ratio";
  Boolean locked "True when the regularized clutch is inside the near-stick slip band";

equation
  assert(T_capacity_max >= 0, "Differential1D: T_capacity_max must be non-negative");

  w_in = der(shaft_in.phi);
  w_l = der(shaft_left.phi);
  w_r = der(shaft_right.phi);
  dw = w_l - w_r;

  // Ideal open-diff kinematics.
  shaft_in.phi = (shaft_left.phi + shaft_right.phi)/2;

  T_in = shaft_in.tau;
  T_open = T_in/2;

  lockFraction = 

    if noEvent(driveSideTorqueSign*T_in >= 0) then
      lockFractionAccel
    else
      lockFractionDecel;

  T_lock_capacity_raw = 

    if use_lsd then
      T_preload + 0.5*lockFraction*abs(T_in)
    else
      0;
  T_lock_capacity = noEvent(min(T_lock_capacity_raw, T_capacity_max));
  T_lock_kinetic_capacity = kineticFrictionRatio*T_lock_capacity;
  clutchNormalForceEquivalent = T_lock_capacity/clutchEffectiveRadius;
  locked = use_lsd and noEvent(abs(dw) <= w_transition);

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

  annotation(Documentation(info = "<html>
<p>
Pure 1D variant of the BobLib differential equations for use inside
VehicleInterfaces driveline adapters. The MultiBody mount support is omitted
intentionally because the surrounding VehicleInterfaces architecture is 1D at
the driveline boundary.
</p>
</html>"));
end Differential1D;
