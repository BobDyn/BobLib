within BobLib.Transmissions.Internal;
model VariableRatioGear

  "Reusable ideal gear with a runtime-selectable ratio"
  import SI = Modelica.Units.SI;

  Modelica.Mechanics.Rotational.Interfaces.Flange_a inputFlange
    "Input shaft" annotation(
      Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b outputFlange
    "Output shaft" annotation(
      Placement(transformation(extent = {{90, -10}, {110, 10}})));
  Modelica.Blocks.Interfaces.RealInput ratio
    "Selected gear ratio (input speed divided by output speed)" annotation(
      Placement(transformation(origin = {0, 110}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

  SI.AngularVelocity w_in "Input shaft speed";
  SI.AngularVelocity w_out "Output shaft speed";

equation
  w_in = der(inputFlange.phi);
  w_out = der(outputFlange.phi);

  // Speed-level ideal gear relation. Constraining velocity rather than
  // position means a discrete ratio change does not impose a non-physical
  // position jump on the connected shafts.
  w_in = ratio*w_out;
  0 = ratio*inputFlange.tau + outputFlange.tau;

  annotation(Documentation(info = "<html>
<p>
Model <code>VariableRatioGear</code> is the reusable gear core used by
transmission adapters that change ratio at run time. The power-conserving
torque relation <code>0 = ratio*inputFlange.tau + outputFlange.tau</code>
pairs with the velocity-level speed relation so the selected ratio can step
between discrete gears without a position discontinuity.
</p>
</html>"));
end VariableRatioGear;
