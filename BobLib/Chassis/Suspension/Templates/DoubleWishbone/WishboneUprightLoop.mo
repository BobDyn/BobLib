within BobLib.Chassis.Suspension.Templates.DoubleWishbone;

model WishboneUprightLoop "Kinematic loop consisting of upright, lower wishbone, and upper wishbone"

  extends BobLib.Icons.WishboneUprightLoopIcon;

  import SI = Modelica.Units.SI;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoopRecord;

  // Record parameters
  parameter WishboneUprightLoopRecord pDW;

  // Visual parameters
  parameter SI.Length linkDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  parameter SI.Length jointDiameter annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  outer parameter Boolean headless;

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upperFrame_i annotation(
    Placement(
      transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lowerFrame_i annotation(
    Placement(
      transformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, -70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b upperFrame_o annotation(
    Placement(
      transformation(origin = {0, 100}, extent = {{16, -16}, {-16, 16}}, rotation = -90),
      iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b lowerFrame_o annotation(
    Placement(
      transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b steeringFrame annotation(
    Placement(
      transformation(origin = {100, -60}, extent = {{16, -16}, {-16, 16}}),
      iconTransformation(origin = {100, -70}, extent = {{-16, -16}, {16, 16}})));

  // Upper wishbone + upright
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUSR upperWishboneUpright(
    n1_a = {1, 0, 0},
    n_b = Vectors.normalize(pDW.upperFore_i - pDW.upperAft_i),
    rRod1_ia = pDW.upper_o - pDW.lower_o,
    rRod2_ib = pDW.upper_o - (pDW.upperFore_i + pDW.upperAft_i)/2,
    sphereDiameter = jointDiameter,
    rod1Diameter = linkDiameter,
    rod2Diameter = 0,
    revoluteDiameter = linkDiameter,
    revoluteLength = Vectors.norm(pDW.upperFore_i - pDW.upperAft_i) + linkDiameter,
    cylinderLength = jointDiameter*0.125,
    cylinderDiameter = jointDiameter*0.125,
    animation = not headless) annotation(
    Placement(transformation( extent = {{20, -20}, {-20, 20}}, rotation = -90)));

  // Lower wisbone
  Modelica.Mechanics.MultiBody.Joints.Revolute lowerJoint_i(
    n = Vectors.normalize(pDW.lowerFore_i - pDW.lowerAft_i),
    cylinderLength = Vectors.norm(pDW.lowerFore_i - pDW.lowerAft_i) + linkDiameter,
    cylinderDiameter = linkDiameter,
    animation = not headless,
    stateSelect = StateSelect.always,
    phi(nominal = 0.05),
    w(start = 0, nominal = 1)) annotation(
    Placement(transformation(origin = {-70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerLink(
    r = pDW.lower_o - (pDW.lowerFore_i + pDW.lowerAft_i)/2,
    width = linkDiameter,
    height = linkDiameter,
    animation = false) annotation(
    Placement(transformation(origin = {-30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  // Steering interface
  Modelica.Mechanics.MultiBody.Joints.Revolute steeringAxis(
    n = Vectors.normalize(pDW.upper_o - pDW.lower_o),
    cylinderLength = jointDiameter,
    cylinderDiameter = jointDiameter,
    animation = not headless,
    phi(nominal = 0.1),
    w(start = 0, nominal = 1)) annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  // Visualization
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upperFrameToFore(
    r = (pDW.upperFore_i - pDW.upperAft_i)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upperFrameToAft(
    r = (pDW.upperAft_i - pDW.upperFore_i)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape upperForeRod(
    shapeType = "cylinder",
    lengthDirection = Frames.resolve2(upperFrameToFore.frame_b.R, upperForeLinkDirection),
    length = upperForeLinkLength,
    width = linkDiameter,
    height = linkDiameter,
    color = {0, 0, 0},
    animation = not headless) annotation(
    Placement(transformation(origin = {-40, 90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape upperAftRod(
    shapeType = "cylinder",
    lengthDirection = Frames.resolve2(upperFrameToAft.frame_b.R, upperAftLinkDirection),
    length = upperAftLinkLength,
    width = linkDiameter,
    height = linkDiameter,
    color = {0, 0, 0},
    animation = not headless) annotation(
    Placement(transformation(origin = {-40, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerFrameToFore(
    r = (pDW.lowerFore_i - pDW.lowerAft_i)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerFrameToAft(
    r = (pDW.lowerAft_i - pDW.lowerFore_i)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape lowerForeRod(
    shapeType = "cylinder",
    lengthDirection = Frames.resolve2(lowerFrameToFore.frame_b.R, lowerForeLinkDirection),
    length = lowerForeLinkLength,
    width = linkDiameter,
    height = linkDiameter,
    color = {0, 0, 0},
    animation = not headless) annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape lowerAftRod(
    shapeType = "cylinder",
    lengthDirection = Frames.resolve2(lowerFrameToAft.frame_b.R, lowerAftLinkDirection),
    length = lowerAftLinkLength,
    width = linkDiameter,
    height = linkDiameter,
    color = {0, 0, 0},
    animation = not headless) annotation(
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
    Line(points = {{-100, 60}, {0, 60}, {0, 20}}));
  connect(lowerFrame_i, lowerJoint_i.frame_a) annotation(
    Line(points = {{-100, -60}, {-80, -60}}));
  connect(lowerJoint_i.frame_b, lowerLink.frame_a) annotation(
    Line(points = {{-60, -60}, {-40, -60}}, color = {95, 95, 95}));
  connect(lowerLink.frame_b, upperWishboneUpright.frame_a) annotation(
    Line(points = {{-20, -60}, {0, -60}, {0, -20}}, color = {95, 95, 95}));
  connect(upperWishboneUpright.frame_ia, steeringAxis.frame_a) annotation(
    Line(points = {{20, -16}, {30, -16}, {30, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(steeringAxis.frame_b, steeringFrame) annotation(
    Line(points = {{60, -60}, {100, -60}}, color = {95, 95, 95}));
  connect(upperWishboneUpright.frame_im, upperFrame_o) annotation(
    Line(points = {{20, 0}, {20, -0.25}, {40, -0.25}, {40, 80}, {0, 80}, {0, 100}}, color = {95, 95, 95}));
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
    Diagram,
    Documentation(info = "<html>
<p>
Model <code>WishboneUprightLoop</code> assembles one double-wishbone corner kinematic loop.
</p>
<p>
It connects upper and lower control arms, upright, tie-rod pickup, wheel center, and chassis hardpoints from the corresponding record.
</p>
</html>"));
end WishboneUprightLoop;
