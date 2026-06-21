within BobLibVehicleInterfaces.Atmospheres;
package Interfaces

  "BobLib atmosphere bus interfaces"
  extends Modelica.Icons.InterfacesPackage;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Atmospheres.Interfaces</code> contains
BobLib-specific signal-bus interfaces used to expose atmosphere measurements
through the shared VehicleInterfaces control bus. VehicleInterfaces 2.0.2
atmosphere models define replaceable functions but no control-bus connector, so
BobLib adds this nested bus only where atmosphere values need to be broadcast to
other subsystems.
</p>
</html>"));
end Interfaces;
