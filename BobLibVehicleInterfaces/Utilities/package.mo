within BobLibVehicleInterfaces;
package Utilities

  "General math and FMI helpers that do not map onto VehicleInterfaces roots"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageBackground;

  annotation(uses(Modelica(version = "4.1.0")),
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Utilities</code> contains shared utilities that do not map to a VehicleInterfaces subsystem root.
</p>
<p>
Math, mechanics, and FMI helpers are kept here so domain packages can stay focused on vehicle physics and adapters.
</p>
</html>"));
end Utilities;
