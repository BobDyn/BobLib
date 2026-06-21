within BobLibVehicleInterfaces.Icons;

partial model InverterDCIcon "Reusable inverter dc icon"

  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {35, 35, 35}, fillColor = {240, 243, 248}, fillPattern = FillPattern.Solid, extent = {{-80, 80}, {80, -80}}), Line(points = {{-54, -24}, {-18, 24}, {18, -24}, {54, 24}}, color = {30, 90, 150}, thickness = 1), Text(textColor = {32, 32, 32}, extent = {{-66, 64}, {66, 44}}, textString = "%name"), Text(textColor = {35, 35, 35}, extent = {{-42, 10}, {42, -10}}, textString = "DC/AC")}),
    Documentation(info = "<html>
<p>
Partial model <code>InverterDCIcon</code> defines shared icon geometry for inverter dc icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end InverterDCIcon;