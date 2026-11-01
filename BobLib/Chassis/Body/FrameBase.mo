within BobLib.Chassis.Body;

partial model FrameBase

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Chassis.FrameRecord pFrame "Sprung-frame reference positions, combined mass properties and torsional compliance" annotation(Dialog(group = "Calibration"));

  extends BobLib.Icons.FrameBaseIcon;

  // Visual parameters
  outer parameter SI.Length linkDiameter "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(
    Dialog(tab = "Animation"));
  outer parameter SI.Length jointDiameter "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(
    Dialog(tab = "Animation"));
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frontFrame annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rearFrame annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.MultiBody.Parts.Body sprungBody(
    m = pFrame.mass.m,
    r_CM = {0, 0, 0},
    I_11 = pFrame.mass.inertia[1, 1],
    I_22 = pFrame.mass.inertia[2, 2],
    I_33 = pFrame.mass.inertia[3, 3],
    I_21 = pFrame.mass.inertia[2, 1],
    I_31 = pFrame.mass.inertia[3, 1],
    I_32 = pFrame.mass.inertia[3, 2],
    sphereDiameter = jointDiameter,
    cylinderDiameter = 0,
    useQuaternions = false,
    angles_fixed = false) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToFore(r = (pFrame.frontReference - pFrame.rearReference)/2) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToAft(r = -1*(pFrame.frontReference - pFrame.rearReference)/2) annotation(
    Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b cgFrame annotation(
    Placement(
      transformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0),
      iconTransformation(origin = {0, -60}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToCG(
    r = pFrame.mass.rCM - (pFrame.frontReference + pFrame.rearReference)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(midToFore.frame_b, frontFrame) annotation(
    Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(midToAft.frame_b, rearFrame) annotation(
    Line(points = {{60, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(midToCG.frame_a, midToFore.frame_a) annotation(
    Line(points = {{-20, 60}, {-30, 60}, {-30, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(midToCG.frame_b, cgFrame) annotation(
    Line(points = {{0, 60}, {100, 60}}, color = {95, 95, 95}));
  connect(sprungBody.frame_a, midToCG.frame_b) annotation(
    Line(points = {{40, 30}, {0, 30}, {0, 60}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-100, -80}, {100, 80}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Partial model <code>FrameBase</code> defines the sprung chassis frame interface.
</p>
<p>
Concrete frame models provide rigid or compliant body behavior while exposing the same multibody frame and mass-property contract to the detailed chassis.
</p>
</html>"));
end FrameBase;
