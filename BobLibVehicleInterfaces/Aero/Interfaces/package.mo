within BobLibVehicleInterfaces.Aero;
package Interfaces
  "Interface definitions for aerodynamic subsystems"
  extends Modelica.Icons.InterfacesPackage;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Aero.Interfaces</code> defines aerodynamic connector contracts for BobLibVehicleInterfaces.
</p>
<p>
The aero interface is BobLib-specific and parallels the VehicleInterfaces style: first-level aero models expose only the shared signals and nested models provide the physics.
</p>
</html>"));
end Interfaces;
