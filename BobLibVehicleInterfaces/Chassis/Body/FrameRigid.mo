within BobLibVehicleInterfaces.Chassis.Body;

model FrameRigid
  extends BobLibVehicleInterfaces.Chassis.Body.FrameBase;

equation
  connect(midToAft.frame_a, midToFore.frame_a) annotation(
    Line(points = {{40, 0}, {-40, 0}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    Icon(graphics = {Line(points = {{-20, 0}, {20, 0}}, thickness = 5)}),
    Documentation(info = "<html>
<p>
Model <code>FrameRigid</code> represents the sprung chassis as a rigid multibody body.
</p>
<p>
It uses the supplied mass, center of mass, and inertia record to connect the chassis body into the detailed vehicle assembly.
</p>
</html>"));
end FrameRigid;
