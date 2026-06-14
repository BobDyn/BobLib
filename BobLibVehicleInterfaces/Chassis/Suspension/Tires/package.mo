within BobLibVehicleInterfaces.Chassis.Suspension;
package Tires
  "Tire, wheel, slip, and contact-patch frame models used by suspension axles"
  extends BobLibVehicleInterfaces.Icons.BobLibPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Chassis.Suspension.Tires</code> contains tire models under the suspension domain.
</p>
<p>
Tires live with suspension assemblies because axle models own wheel centers,
tire load paths, and raw contact-patch frames. Shared MultiBody contact
mechanics, fixtures, and actuators live under
<code>Utilities.Mechanics.MultiBody</code> and are connected at the chassis or
rig assembly level.
</p>
</html>"));
end Tires;
