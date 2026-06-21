within BobLibVehicleInterfaces;
package Controllers

  "Controller models extending VehicleInterfaces controller contracts"
  extends Modelica.Icons.Package;
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLibVehicleInterfaces.Controllers</code> contains controller models exposed at the VehicleInterfaces controller boundary.
</p>
<p>
The public VCU model adapts user commands, motor feedback, and HV bus feedback into the power request and limited torque command used by the vehicle stack.
</p>
</html>"));
end Controllers;
