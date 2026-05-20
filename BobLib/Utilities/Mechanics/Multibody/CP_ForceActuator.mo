within BobLib.Utilities.Mechanics.Multibody;

model CP_ForceActuator

  parameter Real fxTable[:, 2] "Time-series table for Fx, with 1 indicating active and 0 indicating inactive";
  parameter Real fyTable[:, 2] "Time-series table for Fy, with 1 indicating active and 0 indicating inactive";
  parameter Real forceMagnitude;
  
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
  Modelica.Mechanics.MultiBody.Forces.WorldForce worldForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world)  annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));
  
  // Output
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassisFrame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 180)));

equation
  connect(leftFxSource.y[1], leftFxGain.u) annotation(
    Line(points = {{-59, 30}, {-31, 30}}, color = {0, 0, 127}));
  connect(leftFySource.y[1], leftFyGain.u) annotation(
    Line(points = {{-59, 0}, {-30, 0}}, color = {0, 0, 127}));
  connect(leftFxGain.y, worldForce.force[1]) annotation(
    Line(points = {{-20, 30}, {0, 30}, {0, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(leftFyGain.y, worldForce.force[2]) annotation(
    Line(points = {{-20, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(leftFzSource.y, worldForce.force[3]) annotation(
    Line(points = {{-58, -30}, {0, -30}, {0, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(worldForce.frame_b, chassisFrame) annotation(
    Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}));

annotation(
    Icon(graphics = {Text(origin = {0, 2},extent = {{-60, 40}, {60, -40}}, textString = "Fx, Fy, Fz"), Rectangle(lineThickness = 1, extent = {{-62, 20}, {62, -20}}), Text(origin = {0, -40}, extent = {{-100, 20}, {100, -20}}, textString = "%name"), Line(origin = {78, 0}, points = {{-16, 0}, {14, 0}}, thickness = 3, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {0, 40}, extent = {{-100, 20}, {100, -20}}, textString = "%forceMagnitude")}));
end CP_ForceActuator;