within BobLibVehicleInterfaces.Icons;

partial model TabularDamperIcon "Reusable tabular damper icon"

  annotation(
    Icon(graphics = {Line(origin = {-60, 0}, points = {{-40, 0}, {40, 0}}, thickness = 5), Line(origin = {-20, 0}, points = {{0, 36}, {0, -36}}, thickness = 5), Line(origin = {-5, 0}, points = {{-25, 40}, {25, 40}, {25, -40}, {-25, -40}}, thickness = 5), Line(origin = {65, 0}, points = {{-45, 0}, {35, 0}}, thickness = 5), Line(origin = {-0.13, -19.13}, points = {{-59.8675, -12.8675}, {-49.8675, 17.1325}, {-19.8675, 43.1325}, {60.1325, 47.1325}}, color = {255, 0, 0}, thickness = 3)}),
    Documentation(info = "<html>
<p>
Partial model <code>TabularDamperIcon</code> defines shared icon geometry for tabular damper icon.
</p>
<p>
It contains only graphical annotation primitives. Component and package models extend it to share the BobLib visual language without duplicating icon geometry.
</p>
</html>"));
end TabularDamperIcon;
