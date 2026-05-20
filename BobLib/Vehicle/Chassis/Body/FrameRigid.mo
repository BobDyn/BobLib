within BobLib.Vehicle.Chassis.Body;

model FrameRigid
  extends BobLib.Vehicle.Chassis.Body.FrameBase;

equation
  connect(midToAft.frame_a, midToFore.frame_a) annotation(
    Line(points = {{40, 0}, {-40, 0}}, color = {95, 95, 95}));
  annotation(
    Diagram(graphics),
    Icon(graphics = {Line(points = {{-20, 0}, {20, 0}}, thickness = 5)}));
end FrameRigid;
