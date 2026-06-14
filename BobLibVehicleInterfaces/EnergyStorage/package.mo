within BobLibVehicleInterfaces;
package EnergyStorage
  "Energy-storage models extending VehicleInterfaces energy-storage contracts"
  extends Modelica.Icons.Package;
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.EnergyStorage</code> contains energy-storage models for the VehicleInterfaces-based vehicle stack.
</p>
<p>
The public battery pack adapter exposes pack voltage, current, and state-of-charge while nested models hold the BobLib battery physics.
</p>
</html>"));
end EnergyStorage;
