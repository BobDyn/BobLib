within BobLibVehicleInterfaces.Icons;

partial model RackAndPinionIcon "Reusable rack and pinion icon"

  annotation(
    Icon(graphics = {
      Line(origin = {-25.8, 3.2}, points = {{-54.2, -3.2}, {25.8, -3.2}, {105.8, -3.2}}, thickness = 15)
    }),
    Documentation(info = "<html>
<p>
Partial model <code>RackAndPinionIcon</code> defines shared icon geometry for rack and pinion icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end RackAndPinionIcon;
