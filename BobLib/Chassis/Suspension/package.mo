within BobLib.Chassis;
package Suspension

  "Suspension, axle, tire, and contact-patch frame physics"
  extends BobLib.Icons.BobLibPackageBackground;

  annotation(Documentation(info = "<html>
<p>
Suspension-domain physics for the BobLib chassis adapter. Axle assemblies and
linkages live directly in this package.
Tire, wheel, slip, and contact-patch models live in <code>Tires</code> because
they are owned by the suspension/axle loop rather than by a separate
VehicleInterfaces subsystem contract. Reusable MultiBody actuators, fixtures,
and contact mechanics that close contact-patch frames to rig components live in
<code>Utilities.Mechanics.MultiBody.ContactMechanics</code>.
</p>
</html>"));
end Suspension;
