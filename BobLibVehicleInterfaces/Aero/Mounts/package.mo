within BobLibVehicleInterfaces.Aero;
package Mounts
  "Aerodynamic load-frame mount adapters"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Aero.Mounts</code> contains aerodynamic load mounting models.
</p>
<p>
Mounts translate aero force and torque outputs into chassis-frame load application points while keeping the map model independent of chassis geometry.
</p>
</html>"));
end Mounts;
