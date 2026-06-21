within BobLibVehicleInterfaces.Aero.Mounts;

model RigidMount

  "Rigid aero load-frame mount from the sprung chassis frame"
  import SI = Modelica.Units.SI;

  parameter SI.Position r[3] = {0, 0, 0}
    "Vector from sprung chassis frame to aero load frame, resolved in sprung chassis frame";
  parameter Boolean animation = false
    "Enable MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a sprungChassisFrame
    "Sprung chassis-side frame" annotation(
      Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b aeroFrame
    "Aero load frame" annotation(
      Placement(transformation(extent = {{84, -16}, {116, 16}})));

protected
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(
    r = r,
    animation = animation) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}})));

equation
  connect(sprungChassisFrame, fixedTranslation.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, aeroFrame) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(extent = {{-120, -80}, {120, 80}})),
    Icon(coordinateSystem(extent = {{-120, -80}, {120, 80}}), graphics = {
      Rectangle(extent = {{-80, 20}, {80, -20}}, lineColor = {95, 95, 95}, fillColor = {235, 235, 235}, fillPattern = FillPattern.Solid),
      Line(points = {{-100, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}, thickness = 1),
      Text(extent = {{-70, 60}, {70, 25}}, textString = "%name")}),
    Documentation(info = "<html>
<p>
Rigid fixed-translation mount from the sprung chassis reference frame to the aero load
application frame. This is the initial aero mount adapter; compliant or
multi-point mount variants can follow the same frame contract later.
</p>
</html>"));
end RigidMount;
