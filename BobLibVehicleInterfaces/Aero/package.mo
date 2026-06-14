within BobLibVehicleInterfaces;
package Aero
  "Aerodynamic interfaces and chassis mount adapters"
  extends Modelica.Icons.Package;

  annotation(
    Documentation(info = "<html>
<p>
This package defines the public aerodynamic subsystem contract used by the
vehicle assemblies. The aero interface is BobLib-specific, but it follows the
VehicleInterfaces pattern: a small public subsystem boundary, one direct
experiment-facing subsystem model, and helper implementations nested below the
domain package.
</p>
</html>"));
end Aero;
