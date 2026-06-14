within BobLibVehicleInterfaces.Utilities;
package FMI
  extends BobLibVehicleInterfaces.Icons.FMIIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Utilities.FMI</code> contains FMI-oriented utilities.
</p>
<p>
Keeping FMI support separate prevents export helpers from mixing with vehicle physics or VehicleInterfaces adapters.
</p>
</html>"));
end FMI;
