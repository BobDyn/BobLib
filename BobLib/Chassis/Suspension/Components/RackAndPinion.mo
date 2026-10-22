within BobLib.Chassis.Suspension.Components;

model RackAndPinion "Rack and pinion"

  import Modelica.Units.SI;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.SteeringRack.RackAndPinionRecord;

  extends BobLib.Icons.RackAndPinionIcon;

  parameter Boolean steerable = true "Enable steering input; otherwise retain the linkage at its locked rack position" annotation(Dialog(group = "Architecture"), Evaluate = true, choices(checkBox = true));
  parameter SI.Position lockedRackPosition = 0 "Rack displacement held when steering is disabled, measured from its neutral position" annotation(Dialog(group = "Operating settings", enable = not steerable));

  // Record parameters
  parameter RackAndPinionRecord pRack "Rack pickup geometry and pinion-to-rack motion conversion" annotation(Dialog(group = "Calibration"));

  // Visual parameters
  parameter SI.Length linkDiameter "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(
    Evaluate = true, Dialog(tab = "Animation"));
  outer parameter Boolean headless "Disable MultiBody animation geometry for simulation without visualization";

  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame annotation(
    Placement(
      transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -20}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b leftFrame annotation(
    Placement(
      transformation(origin = {-140, 50}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rightFrame annotation(
    Placement(
      transformation(origin = {140, 50}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));

  // Rack axis
  Modelica.Mechanics.MultiBody.Joints.Prismatic rackAxis(n = {0, 1, 0},
                                                         useAxisFlange = true,
                                                         animation = not headless,
                                                         s(nominal = 0.01)) annotation(
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fromLeft(
    r = {0, -pRack.leftPickup[2], 0},
    width = linkDiameter,
    height = linkDiameter,
    animation = not headless) annotation(
    Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRight(
    r = {0, -pRack.leftPickup[2], 0},
    width = linkDiameter,
    height = linkDiameter,
    animation = not headless) annotation(
    Placement(transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a pinionFlange annotation(
    Placement(
      transformation(origin = {-140, -20}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D annotation(
    Placement(transformation(origin = {-50, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  // Visualization
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape pinionVisualizer(
    length = 2*pRack.cFactor/(2*Modelica.Constants.pi),
    width = 1.25*pRack.cFactor/(2*Modelica.Constants.pi),
    shapeType = "cylinder",
    lengthDirection = {0, 0, 1},
    height = 1.25*pRack.cFactor/(2*Modelica.Constants.pi),
    color = {255, 255, 255},
    animation = not headless) annotation(
    Placement(transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Visualizers.FixedShape travelVisualizer(
    shapeType = "cylinder",
    lengthDirection = {0, 1, 0},
    widthDirection = {0, 0, 1},
    length = linkDiameter,
    width = linkDiameter*1.1,
    height = linkDiameter*1.1,
    color = {0, 0, 0},
    r_shape = {0, -linkDiameter/2, 0},
    animation = not headless) annotation(
    Placement(transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

  Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(
    useSupportR = true,
    useSupportT = true,
    ratio = 2*Modelica.Constants.pi/pRack.cFactor) annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-15, -15}, {15, 15}})));

protected
  Modelica.Mechanics.Rotational.Sources.Position rackLock(useSupport = true, exact = true) if not steerable annotation(
    Placement(visible = not steerable, transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.Constant lockPosition(k = lockedRackPosition*2*Modelica.Constants.pi/pRack.cFactor) if not steerable annotation(
    Placement(visible = not steerable, transformation(origin = {-125, -50}, extent = {{-6, -6}, {6, 6}})));

equation
  connect(lockPosition.y, rackLock.phi_ref) annotation(
    Line(visible = not steerable, points = {{-118.4, -50}, {-112, -50}}, color = {0, 0, 127}));
  connect(rackLock.flange, pinionFlange) annotation(
    Line(visible = not steerable, points = {{-90, -50}, {-90, -20}, {-140, -20}}));
  connect(rackLock.support, mounting1D.flange_b) annotation(
    Line(
      visible = not steerable,
      points = {{-100, -60}, {-75, -60}, {-75, -70}, {-50, -70}}));
  connect(mountFrame, rackAxis.frame_a) annotation(
    Line(points = {{0, -100}, {0, -10}}));
  connect(fromLeft.frame_b, rackAxis.frame_b) annotation(
    Line(points = {{-20, 50}, {0, 50}, {0, 10}}, color = {95, 95, 95}));
  connect(toRight.frame_a, fromLeft.frame_b) annotation(
    Line(points = {{20, 50}, {-20, 50}}, color = {95, 95, 95}));
  connect(pinionVisualizer.frame_a, rackAxis.frame_a) annotation(
    Line(points = {{20, -70}, {0, -70}, {0, -10}}, color = {95, 95, 95}));
  connect(mounting1D.frame_a, rackAxis.frame_a) annotation(
    Line(points = {{-60, -80}, {0, -80}, {0, -10}}, color = {95, 95, 95}));
  connect(travelVisualizer.frame_a, rackAxis.frame_b) annotation(
    Line(points = {{20, 20}, {0, 20}, {0, 10}}, color = {95, 95, 95}));
  connect(pinionFlange, idealGearR2T.flangeR) annotation(
    Line(points = {{-140, -20}, {-100, -20}, {-100, 0}, {-75, 0}}));
  connect(idealGearR2T.supportR, mounting1D.flange_b) annotation(
    Line(points = {{-75, -15}, {-75, -70}, {-50, -70}}));
  connect(fromLeft.frame_a, leftFrame) annotation(
    Line(points = {{-40, 50}, {-140, 50}}, color = {95, 95, 95}));
  connect(toRight.frame_b, rightFrame) annotation(
    Line(points = {{40, 50}, {140, 50}}, color = {95, 95, 95}));
  connect(idealGearR2T.supportT, rackAxis.support) annotation(
    Line(points = {{-45, -15}, {-20, -15}, {-20, -4}, {-6, -4}}, color = {0, 127, 0}));
  connect(idealGearR2T.flangeT, rackAxis.axis) annotation(
    Line(points = {{-45, 0}, {-20, 0}, {-20, 8}, {-6, 8}}, color = {0, 127, 0}));
  annotation(Diagram(coordinateSystem(extent = {{-140, -100}, {140, 100}})),
    Icon(graphics = {
      Line(origin = {-80, 0}, points = {{0, 0}, {-10, 0}, {-10, 0}}, color = {255, 0, 0}, thickness = 10),
      Line(origin = {100, 0}, points = {{-10, 0}, {-10, 0}, {-20, 0}}, color = {255, 0, 0}, thickness = 10)
    }),
    Documentation(info = "<html>
<p>
Model <code>RackAndPinion</code> converts steering shaft rotation into rack and tie-rod motion.
</p>
<p>
Axle models use it to drive left and right steering pickups from the VehicleInterfaces steering input while preserving the multibody rack support path.
</p>
</html>"));
end RackAndPinion;
