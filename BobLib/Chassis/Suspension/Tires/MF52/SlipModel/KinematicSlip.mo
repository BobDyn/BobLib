within BobLib.Chassis.Suspension.Tires.MF52.SlipModel;

model KinematicSlip

  extends BaseSlipModel;

  parameter Real kappa_max(min = 0) = 2.0 "Clamp for longitudinal slip" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Angle alpha_max(min = 0) = 1.2 "Clamp for slip angle [rad]" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Velocity V_min(min = Modelica.Constants.small) = 0.5 "Low-speed regularization velocity" annotation(Dialog(group = "Advanced"));

protected
  Modelica.Units.SI.Velocity V_ref "Composite reference speed";
  Modelica.Units.SI.Velocity V_planar "Planar contact speed";
  Modelica.Units.SI.Velocity Vsx "Longitudinal slip velocity";
  Modelica.Units.SI.Angle alpha_raw;
  Real kappa_raw;
  Real blend;

equation
  assert(
    V_min > 0 and kappa_max >= 0 and alpha_max >= 0,
    "KinematicSlip: positive V_min and nonnegative slip limits required");
  V_planar = sqrt(Vx^2 + Vy^2);
  V_ref = sqrt(Vx^2 + (R0*omega)^2 + V_min^2);

  blend = V_planar / (V_planar + V_min);

  alpha_raw = blend * atan2(-Vy, sqrt(Vx^2 + V_min^2));

  Vsx = Vx - R0*omega;
  kappa_raw = -Vsx / V_ref;

  alpha = max(min(alpha_raw, alpha_max), -alpha_max);
  kappa = max(min(kappa_raw, kappa_max), -kappa_max);

  annotation(
    Documentation(info = "<html>
<p>
Model <code>KinematicSlip</code> computes tire slip algebraically from wheel and contact-patch kinematics.
</p>
<p>
It provides a lightweight slip representation for simulations that do not need relaxation dynamics.
</p>
</html>"));
end KinematicSlip;
