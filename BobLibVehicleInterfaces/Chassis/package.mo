within BobLibVehicleInterfaces;
package Chassis
  "Chassis-domain models built from VehicleInterfaces chassis contracts"
  extends Modelica.Icons.Package;

  annotation(Documentation(info = "<html>
<p>
Public chassis domain for BobLibVehicleInterfaces. Body, suspension, and
complete chassis models live here because they are part of the VehicleInterfaces
chassis subsystem boundary.
</p>
<p>
Direct chassis models are VehicleInterfaces-facing chassis contract adapters:
partial adapters define the shared boundary, and concrete variants bind records
and detailed physics for vehicle experiments. Detailed implementation cores live
in <code>Internal</code>; body physics lives in <code>Body</code>; suspension,
axle, tire, and raw contact-patch frame models live in <code>Suspension</code>
and its nested packages. Shared MultiBody contact mechanics live under
<code>Utilities.Mechanics.MultiBody</code> and are wired at the chassis or rig
assembly level.
</p>
</html>"));
end Chassis;
