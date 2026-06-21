within BobLibVehicleInterfaces;
package Atmospheres

  "Atmosphere models extending VehicleInterfaces atmosphere contracts"
  extends Modelica.Icons.Package;

  annotation(Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Atmospheres</code> contains atmosphere
models exposed through the standard VehicleInterfaces atmosphere contract.
</p>
<p>
BobLib atmosphere models keep the VehicleInterfaces replaceable functions and
also publish commonly used scalar/vector measurements to a shared
<code>AtmosphereBus</code> so downstream subsystems can subscribe without
direct atmosphere-to-subsystem signal wiring.
</p>
</html>"));
end Atmospheres;
