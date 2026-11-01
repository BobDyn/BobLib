within BobLib.Utilities.Mechanics.MultiBody.Actuators;

model ContactPatchForceActuator

  parameter Real fxTable[:, 2] "Time-series table for Fx, with 1 indicating active and 0 indicating inactive" annotation(Dialog(group = "Operating settings"));
  parameter Real fyTable[:, 2] "Time-series table for Fy, with 1 indicating active and 0 indicating inactive" annotation(Dialog(group = "Operating settings"));
  parameter Modelica.Units.SI.Force forceMagnitude "Force amplitude multiplying the normalized contact-load protocol" annotation(Dialog(group = "Operating settings"));

  // Sources
  Modelica.Blocks.Sources.CombiTimeTable leftFxSource(table = fxTable,
                                                      columns = {2},
                                                      extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
                                                      smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.CombiTimeTable leftFySource(table = fyTable,
                                                      columns = {2},
                                                      extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
                                                      smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant leftFzSource(k = 0) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));

  // Gains
  Modelica.Blocks.Math.Gain leftFxGain(k = forceMagnitude) annotation(
    Placement(transformation(origin = {-25, 30}, extent = {{-5, -5}, {5, 5}})));

  Modelica.Blocks.Math.Gain leftFyGain(k = forceMagnitude) annotation(
    Placement(transformation(origin = {-25, 0}, extent = {{-5, -5}, {5, 5}})));

  // Force generation
  Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));

  // Output
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));

equation
  connect(leftFxSource.y[1], leftFxGain.u) annotation(
    Line(points = {{-59, 30}, {-31, 30}}, color = {0, 0, 127}));
  connect(leftFySource.y[1], leftFyGain.u) annotation(
    Line(points = {{-59, 0}, {-31, 0}}, color = {0, 0, 127}));
  connect(leftFxGain.y, worldForce.force[1]) annotation(
    Line(points = {{-19.5, 30}, {18, 30}, {18, 0}}, color = {0, 0, 127}));
  connect(leftFyGain.y, worldForce.force[2]) annotation(
    Line(points = {{-19.5, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(leftFzSource.y, worldForce.force[3]) annotation(
    Line(points = {{-59, -30}, {18, -30}, {18, 0}}, color = {0, 0, 127}));
  connect(worldForce.frame_b, chassisFrame) annotation(Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}));

  annotation(
    Documentation(info = "<html>
<p>
Model <code>ContactPatchForceActuator</code> applies prescribed force inputs at
a MultiBody frame, typically a tire contact patch in a rig-level test.
</p>
<p>
It is used by four-post and suspension experiments to excite the tire or
chassis through tabulated contact-patch loads.
</p>
</html>"),
  Icon(graphics = { Rectangle(lineThickness = 1, extent = {{-62, 20}, {62, -20}}), Text(extent = {{-58, 18}, {58, -18}}, textString = "Fx, Fy", textColor = {0, 0, 0}, fontSize = 0), Text(origin = {0, -40}, extent = {{-300, 20}, {300, -20}}, textString = "%name", fontSize = 0), Line(origin = {78, 0}, points = {{-16, 0}, {14, 0}}, thickness = 3, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10)}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true)));
end ContactPatchForceActuator;