within BobLib.Chassis.BaseClasses;

partial model PartialChassisAssembly "Shared front axle, sprung body, and rear axle assembly"

  import Modelica.Units.SI;

  parameter Boolean useRoadSurface = false "Use shared VI road; disabled for prescribed four-post fixtures" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean includeRotorEffects = false
    "Include rotor gyroscopic and carrier-acceleration effects" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  parameter Boolean exposeWheelBearings = false "Expose non-spinning wheel-bearing frames for supported driveline connections" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);
  inner parameter SI.Length linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter SI.Length jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  parameter Boolean flatRoadFastPath = false "Use the horizontal z=0 road specialization; requires a compatible VI road" annotation(Dialog(group = "Advanced"), choices(checkBox = true), Evaluate = true);

protected
  replaceable BobLib.Chassis.Suspension.Interfaces.AxleBase frAxleDW constrainedby BobLib.Chassis.Suspension.Interfaces.AxleBase(
    final exposeWheelBearings = exposeWheelBearings,
    final includeRotorEffects = includeRotorEffects,
    final useRoadSurface = useRoadSurface,
    final flatRoadFastPath = flatRoadFastPath)
    "Front suspension axle" annotation(choicesAllMatching = true,
    Placement(transformation(origin = {-57, -10}, extent = {{-36, -16}, {36, 16}}, rotation = 90)));

  replaceable BobLib.Chassis.Suspension.Interfaces.AxleBase rrAxleDW constrainedby BobLib.Chassis.Suspension.Interfaces.AxleBase(
    final exposeWheelBearings = exposeWheelBearings,
    final includeRotorEffects = includeRotorEffects,
    final useRoadSurface = useRoadSurface,
    final flatRoadFastPath = flatRoadFastPath)
    "Rear suspension axle" annotation(choicesAllMatching = true,
    Placement(transformation(origin = {57, -10}, extent = {{-36, 16}, {36, -16}}, rotation = 90)));
  replaceable BobLib.Chassis.Body.FrameBase spaceFrame constrainedby BobLib.Chassis.Body.FrameBase "Sprung frame/body" annotation(choicesAllMatching = true,
    Placement(transformation(origin = {0, -10}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(frAxleDW.axleFrame, spaceFrame.frontFrame) annotation(
    Line(points = {{-55, -10}, {-20, -10}}, color = {95, 95, 95}));
  connect(spaceFrame.rearFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{20, -10}, {55, -10}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -60}, {100, 40}}, preserveAspectRatio = true)),
    Documentation(info = "<html><p>Owns the two independently replaceable axles and sprung body. The public chassis adapter inherits this assembly so its components remain visible and editable in the chassis diagram. Contact fixtures and VehicleInterfaces connectors belong to the adapter.</p></html>"));
end PartialChassisAssembly;
