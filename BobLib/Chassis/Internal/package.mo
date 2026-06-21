within BobLib.Chassis;
package Internal

  "Detailed chassis implementation cores used by VehicleInterfaces-facing chassis adapters"
  extends BobLib.Icons.BobLibInternalPackageBackground;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Chassis.Internal</code> contains detailed chassis implementation models.
</p>
<p>
The public chassis adapter remains the VehicleInterfaces-facing boundary. Detailed sprung-frame, suspension, tire, and aero-load wiring lives here.
</p>
</html>"));
end Internal;
