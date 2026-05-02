within BobLib.Vehicle.Chassis.Suspension.Templates.DoubleWishbone;

model WishboneUprightLoop "Kinematic loop consisting of upright, lower wishbone, and upper wishbone"
  import Modelica.SIunits;
  import Modelica.Math.Vectors;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoopRecord;
  // Record parameters
  parameter WishboneUprightLoopRecord pDW;
  // Visual parameters
  parameter SIunits.Length linkDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  parameter SIunits.Length jointDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upperFrame_i annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lowerFrame_i annotation(
    Placement(transformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b upperFrame_o annotation(
    Placement(transformation(origin = {0, 100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b lowerFrame_o annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b steeringFrame annotation(
    Placement(transformation(origin = {100, -60}, extent = {{16, -16}, {-16, 16}}), iconTransformation(origin = {100, -70}, extent = {{-16, -16}, {16, 16}})));
  // Upper wishbone + upright
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUSR upperWishboneUpright(n1_a = {1, 0, 0}, n_b = Vectors.normalize(pDW.upperFore_i - pDW.upperAft_i), rRod1_ia = pDW.upper_o - pDW.lower_o, rRod2_ib = pDW.upper_o - (pDW.upperFore_i + pDW.upperAft_i)/2, sphereDiameter = jointDiameter, rod1Diameter = linkDiameter, rod2Diameter = 0, revoluteDiameter = linkDiameter, revoluteLength = Vectors.norm(pDW.upperFore_i - pDW.upperAft_i) + linkDiameter, cylinderLength = jointDiameter*0.125, cylinderDiameter = jointDiameter*0.125) annotation(
    Placement(transformation(origin = {-2, 18}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
  // Lower wisbone
  Modelica.Mechanics.MultiBody.Joints.Revolute lowerJoint_i(n = Vectors.normalize(pDW.lowerFore_i - pDW.lowerAft_i), cylinderLength = Vectors.norm(pDW.lowerFore_i - pDW.lowerAft_i) + linkDiameter, cylinderDiameter = linkDiameter, phi(displayUnit = "rad"), w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerLink(r = pDW.lower_o - (pDW.lowerFore_i + pDW.lowerAft_i)/2, width = linkDiameter, height = linkDiameter, animation = false) annotation(
    Placement(transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  // Steering interface
  Modelica.Mechanics.MultiBody.Joints.Revolute steeringAxis(n = Vectors.normalize(pDW.upper_o - pDW.lower_o), cylinderLength = jointDiameter, cylinderDiameter = jointDiameter, phi(nominal = 0.1), w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  // Visualization
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upperFrameToFore(r = (pDW.upperFore_i - pDW.upperAft_i)/2) annotation(
    Placement(transformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upperFrameToAft(r = (pDW.upperAft_i - pDW.upperFore_i)/2) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape upperForeRod(shapeType = "cylinder", lengthDirection = upperForeLinkDirection, length = upperForeLinkLength, width = linkDiameter, height = linkDiameter, color = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-40, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape upperAftRod(shapeType = "cylinder", lengthDirection = upperAftLinkDirection, length = upperAftLinkLength, width = linkDiameter, height = linkDiameter, color = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerFrameToFore(r = (pDW.lowerFore_i - pDW.lowerAft_i)/2) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerFrameToAft(r = (pDW.lowerAft_i - pDW.lowerFore_i)/2) annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape lowerForeRod(shapeType = "cylinder", lengthDirection = lowerForeLinkDirection, length = lowerForeLinkLength, width = linkDiameter, height = linkDiameter, color = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape lowerAftRod(shapeType = "cylinder", lengthDirection = lowerAftLinkDirection, length = lowerAftLinkLength, width = linkDiameter, height = linkDiameter, color = {0, 0, 0}) annotation(
    Placement(transformation(origin = {-40, -90}, extent = {{-10, -10}, {10, 10}})));
protected
  Real upperForeLinkDirection[3];
  Real upperForeLinkLength;
  Real upperAftLinkDirection[3];
  Real upperAftLinkLength;
  Real lowerForeLinkDirection[3];
  Real lowerForeLinkLength;
  Real lowerAftLinkDirection[3];
  Real lowerAftLinkLength;
equation
  upperForeLinkDirection = Vectors.normalize(upperFrame_o.r_0 - upperFrameToFore.frame_b.r_0);
  upperForeLinkLength = Vectors.norm(upperFrame_o.r_0 - upperFrameToFore.frame_b.r_0);
  upperAftLinkDirection = Vectors.normalize(upperFrame_o.r_0 - upperFrameToAft.frame_b.r_0);
  upperAftLinkLength = Vectors.norm(upperFrame_o.r_0 - upperFrameToAft.frame_b.r_0);
  lowerForeLinkDirection = Vectors.normalize(lowerFrame_o.r_0 - lowerFrameToFore.frame_b.r_0);
  lowerForeLinkLength = Vectors.norm(lowerFrame_o.r_0 - lowerFrameToFore.frame_b.r_0);
  lowerAftLinkDirection = Vectors.normalize(lowerFrame_o.r_0 - lowerFrameToAft.frame_b.r_0);
  lowerAftLinkLength = Vectors.norm(lowerFrame_o.r_0 - lowerFrameToAft.frame_b.r_0);
  connect(upperFrame_i, upperWishboneUpright.frame_b) annotation(
    Line(points = {{-100, 60}, {-100, 38}, {-2, 38}}));
  connect(lowerFrame_i, lowerJoint_i.frame_a) annotation(
    Line(points = {{-100, -60}, {-80, -60}}));
  connect(lowerJoint_i.frame_b, lowerLink.frame_a) annotation(
    Line(points = {{-60, -60}, {-40, -60}}, color = {95, 95, 95}));
  connect(lowerLink.frame_b, upperWishboneUpright.frame_a) annotation(
    Line(points = {{-20, -60}, {-20, -2}, {-2, -2}}, color = {95, 95, 95}));
  connect(upperWishboneUpright.frame_ia, steeringAxis.frame_a) annotation(
    Line(points = {{18, 2}, {30, 2}, {30, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(steeringAxis.frame_b, steeringFrame) annotation(
    Line(points = {{60, -60}, {100, -60}}, color = {95, 95, 95}));
  connect(upperWishboneUpright.frame_im, upperFrame_o) annotation(
    Line(points = {{18, 18}, {18, 80}, {0, 80}, {0, 100}}, color = {95, 95, 95}));
  connect(lowerLink.frame_b, lowerFrame_o) annotation(
    Line(points = {{-20, -60}, {0, -60}, {0, -100}}, color = {95, 95, 95}));
  connect(upperFrameToFore.frame_a, upperFrame_i) annotation(
    Line(points = {{-80, 90}, {-90, 90}, {-90, 60}, {-100, 60}}, color = {95, 95, 95}));
  connect(upperFrameToAft.frame_a, upperFrame_i) annotation(
    Line(points = {{-80, 30}, {-90, 30}, {-90, 60}, {-100, 60}}, color = {95, 95, 95}));
  connect(upperFrameToFore.frame_b, upperForeRod.frame_a) annotation(
    Line(points = {{-60, 90}, {-50, 90}}, color = {95, 95, 95}));
  connect(upperFrameToAft.frame_b, upperAftRod.frame_a) annotation(
    Line(points = {{-60, 30}, {-50, 30}}, color = {95, 95, 95}));
  connect(lowerFrameToFore.frame_a, lowerFrame_i) annotation(
    Line(points = {{-80, -30}, {-90, -30}, {-90, -60}, {-100, -60}}, color = {95, 95, 95}));
  connect(lowerFrameToAft.frame_a, lowerFrame_i) annotation(
    Line(points = {{-80, -90}, {-90, -90}, {-90, -60}, {-100, -60}}, color = {95, 95, 95}));
  connect(lowerForeRod.frame_a, lowerFrameToFore.frame_b) annotation(
    Line(points = {{-50, -30}, {-60, -30}}, color = {95, 95, 95}));
  connect(lowerAftRod.frame_a, lowerFrameToAft.frame_b) annotation(
    Line(points = {{-50, -90}, {-60, -90}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Line(origin = {-45.8, 73.2}, points = {{-42.2, -3.2}, {45.8, -3.2}, {45.8, -3.2}}, thickness = 5), Line(origin = {-43.8, -66.8}, points = {{-44.2, -3.2}, {45.8, -3.2}, {45.8, -3.2}}, thickness = 5), Ellipse(origin = {0, -2}, lineThickness = 5, extent = {{-20, 20}, {20, -20}}), Ellipse(origin = {0, 70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {0, -70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-0.08, 0}, points = {{-21.9178, 0}, {-9.9178, 60}, {10.0822, 60}, {22.0822, 0}, {10.0822, -60}, {-9.9178, -60}, {-21.9178, 0}, {-21.9178, 0}}, thickness = 5), Line(origin = {6, -87}, points = {{-6, -13}, {-6, 13}, {-6, 13}}), Line(origin = {0, 87}, points = {{0, 13}, {0, -13}, {0, -13}}), Line(origin = {50, -65}, points = {{50, -5}, {-10, -5}, {-10, 5}, {-50, 5}, {-50, 5}}), Ellipse(origin = {-88, -70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-88, 70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(points = {{0, 60}, {0, -60}, {0, -60}}, pattern = LinePattern.Dash)}),
    Diagram(graphics));
end WishboneUprightLoop;
