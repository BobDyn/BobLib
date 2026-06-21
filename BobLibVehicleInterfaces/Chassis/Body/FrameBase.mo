within BobLibVehicleInterfaces.Chassis.Body;

partial model FrameBase

  extends BobLibVehicleInterfaces.Icons.FrameBaseIcon;

  import SI = Modelica.Units.SI;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  parameter SI.Position frRef[3];
  parameter SI.Position rrRef[3];

  parameter MassRecord pSprung;

  // Visual parameters
  outer parameter SI.Length linkDiameter annotation(
    Dialog(tab = "Animation"));
  outer parameter SI.Length jointDiameter annotation(
    Dialog(tab = "Animation"));
  outer parameter Boolean headless;

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frontFrame annotation(
    Placement(
      transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rearFrame annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.MultiBody.Parts.Body sprungBody(
    m = pSprung.m,
    r_CM = {0, 0, 0},
    I_11 = pSprung.inertia[1, 1],
    I_22 = pSprung.inertia[2, 2],
    I_33 = pSprung.inertia[3, 3],
    I_21 = pSprung.inertia[2, 1],
    I_31 = pSprung.inertia[3, 1],
    I_32 = pSprung.inertia[3, 2],
    sphereDiameter = jointDiameter,
    cylinderDiameter = 0,
    useQuaternions = false,
    angles_fixed = false) annotation(
    Placement(transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToFore(r = (frRef - rrRef)/2) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToAft(r = -1*(frRef - rrRef)/2) annotation(
    Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b cgFrame annotation(
    Placement(
      transformation(origin = {20, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -60}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation midToCG(
    r = pSprung.rCM - (frRef + rrRef)/2,
    animation = false) annotation(
    Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(midToFore.frame_b, frontFrame) annotation(
    Line(points = {{-60, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(midToAft.frame_b, rearFrame) annotation(
    Line(points = {{60, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(midToCG.frame_a, midToFore.frame_a) annotation(
    Line(points = {{-20, 70}, {-30, 70}, {-30, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(midToCG.frame_b, cgFrame) annotation(
    Line(points = {{0, 70}, {20, 70}, {20, 100}}, color = {95, 95, 95}));
  connect(sprungBody.frame_a, midToCG.frame_b) annotation(
    Line(points = {{40, 70}, {0, 70}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Documentation(info = "<html>
<p>
Partial model <code>FrameBase</code> defines the sprung chassis frame interface.
</p>
<p>
Concrete frame models provide rigid or compliant body behavior while exposing the same multibody frame and mass-property contract to the detailed chassis.
</p>
</html>"));
end FrameBase;
