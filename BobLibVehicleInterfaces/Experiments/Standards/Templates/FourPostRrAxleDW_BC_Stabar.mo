within BobLibVehicleInterfaces.Experiments.Standards.Templates;

model FourPostRrAxleDW_BC_Stabar
  import SI = Modelica.Units.SI;

  extends BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar;

  SI.Length leftSpringLength;
  SI.Length rightSpringLength;
  SI.Angle stabarAngle;

equation
  leftSpringLength = leftShockLinkage.lineForceWithMass.s;
  rightSpringLength = rightShockLinkage.lineForceWithMass.s;
  stabarAngle = stabar.spring.phi_rel;
  annotation(
    Documentation(info = "<html>
<p>
Model <code>FourPostRrAxleDW_BC_Stabar</code> specializes a rear double-wishbone axle with bellcrank-actuated spring/damper motion and a stabilizer bar for the four-post rig.
</p>
<p>
It redeclares tire and wheel choices appropriate for fixture-driven suspension evaluation rather than full vehicle driving.
</p>
</html>"));
end FourPostRrAxleDW_BC_Stabar;
