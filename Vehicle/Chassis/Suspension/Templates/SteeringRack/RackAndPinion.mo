within BobLib.Vehicle.Chassis.Suspension.Templates.SteeringRack;

model RackAndPinion "Rack and pinion"
  import Modelica.SIunits;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.SteeringRack.RackAndPinionRecord;
  // Record parameters
  parameter RackAndPinionRecord pRack;

  // Visual parameters
  parameter SIunits.Length linkDiameter annotation(
    Evaluate = true, Dialog(tab="Animation"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -20}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b leftFrame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rightFrame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  // Rack axis
  Modelica.Mechanics.MultiBody.Joints.Prismatic rackAxis(n = {0, 1, 0},
                                                         useAxisFlange = true,
                                                         animation = false,
                                                         s(nominal=0.01))  annotation(
    Placement(transformation(origin = {0, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fromLeft(r = {0, -pRack.leftPickup[2], 0}, width = linkDiameter, height = linkDiameter)  annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRight(r = {0, -pRack.leftPickup[2], 0}, width = linkDiameter, height = linkDiameter)  annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Mechanics.Rotational.Interfaces.Flange_a pinionFlange annotation(
    Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(
    Placement(transformation(origin = {50, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  // Visualization
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape pinionVisualizer(length = 2*pRack.cFactor/(2*Modelica.Constants.pi), width = 1.25*pRack.cFactor/(2*Modelica.Constants.pi), shapeType = "cylinder", lengthDirection = {0, 0, 1}, height = 1.25*pRack.cFactor/(2*Modelica.Constants.pi), color = {255, 255, 255})  annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape travelVisualizer(shapeType = "cylinder", lengthDirection = {0, 1, 0}, widthDirection = {0, 0, 1}, length = linkDiameter, width = linkDiameter*1.1, height = linkDiameter*1.1, color = {0, 0, 0}, r_shape = {0, -linkDiameter/2, 0})  annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(useSupportR = true, useSupportT = true, ratio = 2*Modelica.Constants.pi/pRack.cFactor)  annotation(
    Placement(transformation(origin = {35, 35}, extent = {{15, -15}, {-15, 15}})));
equation
  connect(mountFrame, rackAxis.frame_a) annotation(
    Line(points = {{0, -100}, {0, -60}}));
  connect(fromLeft.frame_b, rackAxis.frame_b) annotation(
    Line(points = {{-20, 0}, {0, 0}, {0, -40}}, color = {95, 95, 95}));
  connect(toRight.frame_a, fromLeft.frame_b) annotation(
    Line(points = {{20, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(pinionVisualizer.frame_a, rackAxis.frame_a) annotation(
    Line(points = {{-20, -70}, {0, -70}, {0, -60}}, color = {95, 95, 95}));
  connect(mounting1D.frame_a, rackAxis.frame_a) annotation(
    Line(points = {{40, -70}, {0, -70}, {0, -60}}, color = {95, 95, 95}));
  connect(travelVisualizer.frame_a, rackAxis.frame_b) annotation(
    Line(points = {{20, -30}, {0, -30}, {0, -40}}, color = {95, 95, 95}));
  connect(pinionFlange, idealGearR2T.flangeR) annotation(
    Line(points = {{0, 100}, {0, 80}, {60, 80}, {60, 35}, {50, 35}}));
  connect(idealGearR2T.supportR, mounting1D.flange_b) annotation(
    Line(points = {{50, 20}, {50, -60}}));
  connect(fromLeft.frame_a, leftFrame) annotation(
    Line(points = {{-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(toRight.frame_b, rightFrame) annotation(
    Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(idealGearR2T.supportT, rackAxis.support) annotation(
    Line(points = {{20, 20}, {-60, 20}, {-60, -54}, {-6, -54}}, color = {0, 127, 0}));
  connect(idealGearR2T.flangeT, rackAxis.axis) annotation(
    Line(points = {{20, 35}, {-70, 35}, {-70, -42}, {-6, -42}}, color = {0, 127, 0}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Line(origin = {-25.8, 3.2}, points = {{-54.2, -3.2}, {25.8, -3.2}, {105.8, -3.2}}, thickness = 15), Line(origin = {-80, 0}, points = {{0, 0}, {-10, 0}, {-10, 0}}, color = {255, 0, 0}, thickness = 10), Line(origin = {100, 0}, points = {{-10, 0}, {-10, 0}, {-20, 0}}, color = {255, 0, 0}, thickness = 10)}));
end RackAndPinion;
