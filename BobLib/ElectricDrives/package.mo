within BobLib;
package ElectricDrives

  "Electric-drive models extending VehicleInterfaces electric-drive contracts"
  extends Modelica.Icons.Package;
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.ElectricDrives</code> contains electric-drive models for the VehicleInterfaces-based vehicle stack.
</p>
<p>
The public motor adapter is the first-level insertion point; nested models contain BobLib-specific torque and power limiting behavior.
</p>
</html>"));
end ElectricDrives;
