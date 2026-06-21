within BobLib.Utilities;
package FMI

  extends BobLib.Icons.FMIIcon;

  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Utilities.FMI</code> contains FMI-oriented utilities.
</p>
<p>
Keeping FMI support separate prevents export helpers from mixing with vehicle physics or VehicleInterfaces adapters.
</p>
</html>"));
end FMI;
