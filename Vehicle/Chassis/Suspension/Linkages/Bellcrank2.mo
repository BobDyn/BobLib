within BobLib.Vehicle.Chassis.Suspension.Linkages;

model Bellcrank2
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  
  // Geometry parameters
  parameter SIunits.Position pivot[3] "Pivot coordinates" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position pivotAxis[3] "Pivot rotational axis" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position rRod[3] "Coordinates of push/pullrod pickup, resolved in world frame" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position rShock[3] "Coordinates of shock pickup, resolved in world frame" annotation(
    Dialog(group = "Geometry"));
  
  parameter Integer rodPickup(min=1, max=2) = 1 "Push/pullrod pickup mapping, where 1 is the most counter-clockwise pickup about the left bellcrank (generally with the lowest Z coordinate)" annotation(Dialog(group="Pickup Mapping"));
  parameter Integer shockPickup(min=1, max=2) = 2 "Shock pickup mapping, where 1 is the most counter-clockwise pickup about the left bellcrank (generally with the lowest Z coordinate)" annotation(Dialog(group="Pickup Mapping"));
  
  // Visual parameters
  parameter SIunits.Length linkDiameter annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  parameter SIunits.Length jointDiameter annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  
  final parameter SIunits.Position pickup_1[3] = if (rodPickup == 1) then rRod elseif (shockPickup == 1) then rShock else {0, 0, 0};
  final parameter SIunits.Position pickup_2[3] = if (rodPickup == 2) then rRod elseif (shockPickup == 2) then rShock else {0, 0, 0};
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b pickupFrame1 annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b pickupFrame2 annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  // Rotational DOF
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(n = Vectors.normalize(pivotAxis), animation = true, cylinderLength = jointDiameter, cylinderDiameter = jointDiameter, phi(displayUnit = "rad"), w(start = 0, fixed = true), stateSelect = StateSelect.always) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  // Visualization
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape side_1(lengthDirection = Vectors.normalize(pickup_1 - pivot), length = Vectors.norm(pickup_1 - pivot), width = linkDiameter*0.75, height = linkDiameter*0.75, widthDirection = Vectors.normalize(pivotAxis), shapeType = "cylinder") annotation(
    Placement(transformation(origin = {-30, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape side_2(lengthDirection = Vectors.normalize(pickup_2 - pickup_1), widthDirection = Vectors.normalize(pivotAxis), length = Vectors.norm(pickup_2 - pickup_1), width = linkDiameter*0.75, height = linkDiameter*0.75, shapeType = "cylinder") annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape side_3(lengthDirection = Vectors.normalize(pivot - pickup_2), length = Vectors.norm(pivot - pickup_2), width = linkDiameter*0.75, height = linkDiameter*0.75, widthDirection = Vectors.normalize(pivotAxis), shapeType = "cylinder") annotation(
    Placement(transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, animation = false) annotation(
    Placement(transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}})));


protected
  // Kinematics
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toFirstPickup(final r = pickup_1 - pivot, final extra = 0.0, animation = false) annotation(
    Placement(transformation(origin = {-40, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toSecondPickup(final r = pickup_2 - pivot, final extra = 0.0, animation = false) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));

initial equation
  assert(
    rodPickup <> shockPickup,
    "Bellcrank pickup assignments must be unique"
  );

equation
  connect(mountFrame, revolute.frame_a) annotation(
    Line(points = {{-100, 0}, {-80, 0}}));
  connect(pickupFrame1, toFirstPickup.frame_b) annotation(
    Line(points = {{0, -100}, {0, -40}, {-30, -40}}));
  connect(toSecondPickup.frame_b, pickupFrame2) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(toFirstPickup.frame_a, revolute.frame_b) annotation(
    Line(points = {{-50, -40}, {-56, -40}, {-56, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(toSecondPickup.frame_a, revolute.frame_b) annotation(
    Line(points = {{-10, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(side_1.frame_a, revolute.frame_b) annotation(
    Line(points = {{-40, 20}, {-56, 20}, {-56, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(side_2.frame_a, toFirstPickup.frame_b) annotation(
    Line(points = {{-20, -60}, {-30, -60}, {-30, -40}}, color = {95, 95, 95}));
  connect(side_3.frame_a, toSecondPickup.frame_b) annotation(
    Line(points = {{20, 20}, {10, 20}, {10, 0}}, color = {95, 95, 95}));
  connect(body.frame_a, revolute.frame_b) annotation(
    Line(points = {{-40, -20}, {-56, -20}, {-56, 0}, {-60, 0}}, color = {95, 95, 95}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Line(points = {{-80, 0}, {0, -80}, {80, 0}, {2, 0}, {-80, 0}}, thickness = 3), Line(origin = {-90, 0}, points = {{10, 0}, {-10, 0}}), Ellipse(origin = {-80, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, lineThickness = 2, extent = {{-10, 10}, {10, -10}}), Line(origin = {0, -90}, points = {{0, 10}, {0, -10}}), Line(origin = {90, 0}, points = {{-10, 0}, {10, 0}}), Text(origin = {-40, -80}, extent = {{-20, -20}, {20, 20}}, textString = "P1"), Text(origin = {80, -40}, extent = {{-20, -20}, {20, 20}}, textString = "P2")}));
end Bellcrank2;