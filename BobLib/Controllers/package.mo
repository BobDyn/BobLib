within BobLib;
package Controllers

  "Controller models extending VehicleInterfaces controller contracts"
  extends Modelica.Icons.Package;
  annotation(
    Documentation(info = "<html>
<p>
Package <code>BobLib.Controllers</code> contains controller models exposed at the VehicleInterfaces controller boundary.
</p>
<p>
The public <code>VCU</code> model adapts driver intent, motor feedback, chassis
speed, and HV bus feedback into electric-drive, driveline, and mechanical-brake
requests on the shared VehicleInterfaces control bus.
</p>
<p>
<code>StandardVCU</code> extends that adapter with the standard autonomous
vehicle-simulation maneuver commands used by
<code>Experiments.Standards.VehicleSim</code>.
</p>
</html>"));
end Controllers;
