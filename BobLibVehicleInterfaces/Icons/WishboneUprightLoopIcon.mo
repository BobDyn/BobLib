within BobLibVehicleInterfaces.Icons;

partial model WishboneUprightLoopIcon "Reusable wishbone upright loop icon"
  annotation(
    Icon(graphics = {Line(origin = {-45.8, 73.2}, points = {{-42.2, -3.2}, {45.8, -3.2}, {45.8, -3.2}}, thickness = 5), Line(origin = {-43.8, -66.8}, points = {{-44.2, -3.2}, {45.8, -3.2}, {45.8, -3.2}}, thickness = 5), Ellipse(origin = {0, -2}, lineThickness = 5, extent = {{-20, 20}, {20, -20}}), Ellipse(origin = {0, 70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {0, -70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-0.08, 0}, points = {{-21.9178, 0}, {-9.9178, 60}, {10.0822, 60}, {22.0822, 0}, {10.0822, -60}, {-9.9178, -60}, {-21.9178, 0}, {-21.9178, 0}}, thickness = 5), Line(origin = {6, -87}, points = {{-6, -13}, {-6, 13}, {-6, 13}}), Line(origin = {0, 87}, points = {{0, 13}, {0, -13}, {0, -13}}), Line(origin = {50, -65}, points = {{50, -5}, {-10, -5}, {-10, 5}, {-50, 5}, {-50, 5}}), Ellipse(origin = {-88, -70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-88, 70}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(points = {{0, 60}, {0, -60}, {0, -60}}, pattern = LinePattern.Dash)}),
    Documentation(info = "<html>
<p>
Partial model <code>WishboneUprightLoopIcon</code> defines shared icon geometry for wishbone upright loop icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end WishboneUprightLoopIcon;
