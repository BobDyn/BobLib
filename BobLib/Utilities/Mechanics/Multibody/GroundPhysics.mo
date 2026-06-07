within BobLib.Utilities.Mechanics.Multibody;

model GroundPhysics
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin={-100,0}, extent={{-16,-16},{16,16}}),
    iconTransformation(origin={-100,0}, extent={{-16,-16},{16,16}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin={100,0}, extent={{-16,-16},{16,16}}),
    iconTransformation(origin={0,100}, extent={{-16,-16},{16,16}}, rotation=90)));

  parameter Real c = 100000 "Vertical contact stiffness";
  parameter Real d = 750 "Vertical contact damping";
  parameter Real eps = 1e-6 "Smoothing length";
  parameter Real forceEps = 1.0 "Smoothing force for normal-force clamp";

  Real pen(nominal = 0.01) "Smooth penetration";
  Real r_rel_z(nominal = 0.01) "Relative z displacement, positive when separated";
  Real f_raw(nominal = 1000) "Unclamped normal force";
  Real f_z(nominal = 1000) "Normal force";

equation
  r_rel_z = frame_b.r_0[3] - frame_a.r_0[3];

  // Smooth unilateral penetration:
  // pen ~= max(-r_rel_z, 0)
  pen = smooth(1, 0.5*(sqrt(r_rel_z*r_rel_z + eps*eps) - r_rel_z));

  // Contact force. Damping uses penetration rate, avoiding sensors.
  // Use a smooth positive part to avoid tensile ground force without
  // introducing an initialization discontinuity.
  f_raw = c*pen + d*der(pen);
  f_z = smooth(1, 0.5*(sqrt(f_raw*f_raw + forceEps*forceEps) + f_raw));

  frame_a.f = {0, 0, f_z};
  frame_b.f = -frame_a.f;

  frame_a.t = zeros(3);
  frame_b.t = zeros(3);

  annotation(
    Icon(
      coordinateSystem(extent={{-100,-100},{100,100}}),
      graphics={
        Text(extent={{-120,-160},{120,-120}}, textString="%name", fontSize=14,
          horizontalAlignment=TextAlignment.Center),
        Rectangle(extent={{-100,-100},{100,25}}, fillPattern=FillPattern.Solid,
          fillColor={150,100,50}, lineThickness=2),
        Rectangle(extent={{-100,25},{100,100}}, fillPattern=FillPattern.Solid,
          fillColor={95,200,70}, lineThickness=2)
      }));
end GroundPhysics;
