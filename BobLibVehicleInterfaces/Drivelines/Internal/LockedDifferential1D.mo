within BobLibVehicleInterfaces.Drivelines.Internal;
model LockedDifferential1D
  "1D spool/locked differential with kinematically tied outputs"
  import SI = Modelica.Units.SI;

  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_in
    "Input from final drive" annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_left
    "Left halfshaft output" annotation(
      Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_right
    "Right halfshaft output" annotation(
      Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  SI.AngularVelocity w_in;
  SI.AngularVelocity w_l;
  SI.AngularVelocity w_r;
  SI.AngularVelocity dw;
  SI.Torque T_in;
  SI.Torque T_left_drive;
  SI.Torque T_right_drive;
  SI.Torque T_lock
    "Torque transfer required by the locked kinematic constraint";
  Real lockingValue(unit = "1") "Actual locking value |T_left - T_right|/|T_in|";
  Real torqueBiasRatio(unit = "1") "Approximate high/low output torque ratio";
  Boolean locked "True for the structural spool model";

equation
  w_in = der(shaft_in.phi);
  w_l = der(shaft_left.phi);
  w_r = der(shaft_right.phi);
  dw = w_l - w_r;

  // Structural spool kinematics. Halfshaft compliance remains outside this
  // model, so wheel speeds can still differ transiently through shaft twist.
  shaft_in.phi = shaft_left.phi;
  shaft_in.phi = shaft_right.phi;
  0 = shaft_in.tau + shaft_left.tau + shaft_right.tau;

  T_in = shaft_in.tau;
  T_left_drive = -shaft_left.tau;
  T_right_drive = -shaft_right.tau;
  T_lock = 0.5*(T_left_drive - T_right_drive);

  locked = true;
  lockingValue = noEvent(
    abs(T_left_drive - T_right_drive)/
    max(abs(T_in), 1e-6));
  torqueBiasRatio = noEvent(
    (max(abs(T_left_drive), abs(T_right_drive)) + 1e-6)/
    (min(abs(T_left_drive), abs(T_right_drive)) + 1e-6));

  annotation(Documentation(info = "<html>
<p>
Pure 1D spool/locked differential used when the rear driveline should remove
the limited-slip clutch law and kinematically tie the left and right differential
outputs. Compliant halfshafts remain outside this model.
</p>
</html>"));
end LockedDifferential1D;
