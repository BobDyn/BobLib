within BobLib;
package Transmissions

  "Transmission models extending VehicleInterfaces transmission contracts"
  extends Modelica.Icons.Package;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLib.Transmissions</code> contains
transmission subsystem models exposed through the standard VehicleInterfaces
transmission boundary.
</p>
<p>
Direct models in this package are vehicle-level insertion points. Reusable
gear, shift, clutch, launch, and loss physics should live one package level
deeper, then be adapted here through
<code>VehicleInterfaces.Transmissions.Interfaces.Base</code> or one of its
specialized transmission interfaces.
</p>
</html>"));
end Transmissions;
