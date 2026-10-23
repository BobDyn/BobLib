within BobLib.Experiments.Standards.BaseClasses;

block KnCInputs "Smooth motion protocol for a single axle"

  import Modelica.Units.SI;

  parameter Boolean steerable = true "Generate the repeated steering-angle sweep protocol; otherwise run one locked-axle cycle" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Integer angleCount(
    min = 1,
    max = 7) = 7 "Number of active steering-angle sweeps, using the first entries of steeringAngles (1 to 7)" annotation(Evaluate = false, Dialog(group = "Operating settings", enable = steerable));
  parameter SI.Angle steeringAngles[7] = {-90, -60, -30, 0, 30, 60, 90}*Modelica.Constants.pi/180 "Steering pinion angles for the seven available sweep slots, in radians" annotation(Evaluate = false, Dialog(group = "Operating settings", enable = steerable));
  parameter SI.Length heaveMagnitude = 1.5*0.0254 "Amplitude scaling the normalized vertical-motion protocol" annotation(Dialog(group = "Operating settings"), Evaluate = false);
  parameter SI.Angle rollMagnitude = 1.25*Modelica.Constants.pi/180 "Amplitude scaling the normalized roll-motion protocol, in radians" annotation(Dialog(group = "Operating settings"), Evaluate = false);
  final parameter SI.Time duration = if steerable then 119*(angleCount-1)+118
    else 118 "Total protocol duration including transitions between active steering angles";
  constant Real heaveTable[:, 2] = [0, 0.0; 1, 0.0; 2, -1.0; 3, -1.0; 4, -1.0; 5, -1.0; 6, -1.0; 7, -0.8; 8, -0.8; 9, -0.8; 10, -0.8; 11, -0.8; 12, -0.6; 13, -0.6; 14, -0.6; 15, -0.6; 16, -0.6; 17, -0.4; 18, -0.4; 19, -0.4; 20, -0.4; 21, -0.4; 22, -0.2; 23, -0.2; 24, -0.2; 25, -0.2; 26, -0.2; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.2; 33, 0.2; 34, 0.2; 35, 0.2; 36, 0.2; 37, 0.4; 38, 0.4; 39, 0.4; 40, 0.4; 41, 0.4; 42, 0.6; 43, 0.6; 44, 0.6; 45, 0.6; 46, 0.6; 47, 0.8; 48, 0.8; 49, 0.8; 50, 0.8; 51, 0.8; 52, 1.0; 53, 1.0; 54, 1.0; 55, 1.0; 56, 1.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, 0.0; 64, 0.0; 65, 0.0; 66, 0.0; 67, 0.0; 68, 0.0; 69, 0.0; 70, 0.0; 71, 0.0; 72, 0.0; 73, 0.0; 74, 0.0; 75, 0.0; 76, 0.0; 77, 0.0; 78, 0.0; 79, 0.0; 80, 0.0; 81, 0.0; 82, 0.0; 83, 0.0; 84, 0.0; 85, 0.0; 86, 0.0; 87, 0.0; 88, 0.0; 89, 0.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.0; 94, 0.0; 95, 0.0; 96, 0.0; 97, 0.0; 98, 0.0; 99, 0.0; 100, 0.0; 101, 0.0; 102, 0.0; 103, 0.0; 104, 0.0; 105, 0.0; 106, 0.0; 107, 0.0; 108, 0.0; 109, 0.0; 110, 0.0; 111, 0.0; 112, 0.0; 113, 0.0; 114, 0.0; 115, 0.0; 116, 0.0; 117, 0.0; 118, 0.0];
  constant Real rollTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 0.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 0.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 0.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 0.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 0.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 0.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 0.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 0.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 0.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 0.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, -1.0; 64, -1.0; 65, -1.0; 66, -1.0; 67, -1.0; 68, -0.8; 69, -0.8; 70, -0.8; 71, -0.8; 72, -0.8; 73, -0.6; 74, -0.6; 75, -0.6; 76, -0.6; 77, -0.6; 78, -0.4; 79, -0.4; 80, -0.4; 81, -0.4; 82, -0.4; 83, -0.2; 84, -0.2; 85, -0.2; 86, -0.2; 87, -0.2; 88, 0.0; 89, 0.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.2; 94, 0.2; 95, 0.2; 96, 0.2; 97, 0.2; 98, 0.4; 99, 0.4; 100, 0.4; 101, 0.4; 102, 0.4; 103, 0.6; 104, 0.6; 105, 0.6; 106, 0.6; 107, 0.6; 108, 0.8; 109, 0.8; 110, 0.8; 111, 0.8; 112, 0.8; 113, 1.0; 114, 1.0; 115, 1.0; 116, 1.0; 117, 1.0; 118, 0.0];
  constant Real fxTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 1.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 1.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 1.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 1.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 1.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 1.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 1.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 1.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 1.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 1.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 1.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, 0.0; 64, 0.0; 65, 0.0; 66, 0.0; 67, 0.0; 68, 0.0; 69, 0.0; 70, 0.0; 71, 0.0; 72, 0.0; 73, 0.0; 74, 0.0; 75, 0.0; 76, 0.0; 77, 0.0; 78, 0.0; 79, 0.0; 80, 0.0; 81, 0.0; 82, 0.0; 83, 0.0; 84, 0.0; 85, 0.0; 86, 0.0; 87, 0.0; 88, 0.0; 89, 0.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.0; 94, 0.0; 95, 0.0; 96, 0.0; 97, 0.0; 98, 0.0; 99, 0.0; 100, 0.0; 101, 0.0; 102, 0.0; 103, 0.0; 104, 0.0; 105, 0.0; 106, 0.0; 107, 0.0; 108, 0.0; 109, 0.0; 110, 0.0; 111, 0.0; 112, 0.0; 113, 0.0; 114, 0.0; 115, 0.0; 116, 0.0; 117, 0.0; 118, 0.0];
  constant Real fyTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 0.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 0.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 0.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 0.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 0.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 0.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 0.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 0.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 0.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 0.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, 0.0; 64, 1.0; 65, 0.0; 66, 0.0; 67, 0.0; 68, 0.0; 69, 1.0; 70, 0.0; 71, 0.0; 72, 0.0; 73, 0.0; 74, 1.0; 75, 0.0; 76, 0.0; 77, 0.0; 78, 0.0; 79, 1.0; 80, 0.0; 81, 0.0; 82, 0.0; 83, 0.0; 84, 1.0; 85, 0.0; 86, 0.0; 87, 0.0; 88, 0.0; 89, 1.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.0; 94, 1.0; 95, 0.0; 96, 0.0; 97, 0.0; 98, 0.0; 99, 1.0; 100, 0.0; 101, 0.0; 102, 0.0; 103, 0.0; 104, 1.0; 105, 0.0; 106, 0.0; 107, 0.0; 108, 0.0; 109, 1.0; 110, 0.0; 111, 0.0; 112, 0.0; 113, 0.0; 114, 1.0; 115, 0.0; 116, 0.0; 117, 0.0; 118, 0.0];

  Modelica.Blocks.Interfaces.RealOutput steer(unit = "rad") if steerable annotation(Placement(visible = steerable, transformation(origin = {140, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealOutput heave(unit = "m") annotation(Placement(transformation(origin = {140, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealOutput roll(unit = "rad") annotation(Placement(transformation(origin = {140, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.CombiTimeTable steering(
    table = [0, steeringAngles[1];118, steeringAngles[1];119, steeringAngles[2];237, steeringAngles[2];238, steeringAngles[3];356, steeringAngles[3];357, steeringAngles[4];475, steeringAngles[4];476, steeringAngles[5];594, steeringAngles[5];595, steeringAngles[6];713, steeringAngles[6];714, steeringAngles[7];832, steeringAngles[7]],
    smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative2) if steerable annotation(Placement(visible = steerable, transformation(origin = {-70, 60}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Blocks.Sources.CombiTimeTable heaveSource(
    table = cat(1, heaveTable, [119, 0]),
    smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative2,
    extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic) annotation(Placement(transformation(origin = {-70, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Blocks.Sources.CombiTimeTable rollSource(
    table = cat(1, rollTable, [119, 0]),
    smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative2,
    extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic) annotation(Placement(transformation(origin = {-70, -60}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Blocks.Math.Gain heaveScale(k = heaveMagnitude) annotation(Placement(transformation(origin = {40, 0}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Blocks.Math.Gain rollScale(k = rollMagnitude) annotation(Placement(transformation(origin = {40, -60}, extent = {{-12, -12}, {12, 12}})));

initial equation
  assert(angleCount>=1 and angleCount<=7, "BOBLIB_INVALID_KNC_CONFIG: angleCount must be between 1 and 7.");

equation
  connect(
    steering.y[1],
    steer) annotation(Line(visible = (steerable), points = {{-52.4, 60}, {140, 60}}, color = {0, 0, 127}));
  connect(heaveSource.y[1], heaveScale.u) annotation(Line(points = {{-52.4, 0}, {25.6, 0}}, color = {0, 0, 127}));
  connect(heaveScale.y, heave) annotation(Line(points = {{53.2, 0}, {140, 0}}, color = {0, 0, 127}));
  connect(rollSource.y[1], rollScale.u) annotation(Line(points = {{-52.4, -60}, {25.6, -60}}, color = {0, 0, 127}));
  connect(rollScale.y, roll) annotation(Line(points = {{53.2, -60}, {140, -60}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(extent = {{-100, -60}, {100, 60}}), graphics = {Rectangle(extent = {{-100, -60}, {100, 60}}, lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Line(points = {{-65, -30}, {-40, -30}, {-20, 25}, {15, 25}, {35, -10}, {65, -10}}, color = {0, 0, 127}), Text(extent = {{-100, 90}, {100, 65}}, textString = "%name", textColor = {0, 0, 255})}), Diagram(coordinateSystem(extent = {{-120, -100}, {140, 100}})), Documentation(info = "<html><p>The 118 s heave/roll/load protocol repeats every 119 s, with a one-second neutral steering transition. Monotone cubic motion interpolation provides the second derivatives required by exact position actuators. Steering angles are pinion angles; set calibration before initialization. The enclosing rig terminates at duration.</p></html>"));
end KnCInputs;
