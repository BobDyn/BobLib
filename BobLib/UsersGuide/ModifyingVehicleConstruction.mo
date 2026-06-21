within BobLib.UsersGuide;
class ModifyingVehicleConstruction

  "How to replace vehicle-level subsystems and architectures"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Vehicle-Level Responsibility</h4>
<p>
Vehicle templates assemble subsystems; they should not hide subsystem physics
or detailed control logic. The full-vehicle template
<a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.BaseVehicleSim'>BaseVehicleSim</a>
connects physical ports, connects the shared bus, instantiates road,
atmosphere, aero, and world, and provides standard outputs and termination
monitors. The controller behavior belongs in
<a href='modelica://BobLib.Controllers.StandardVCU'>StandardVCU</a> or another
VCU model.
</p>
<h4>VehicleInterfaces Boundary</h4>
<p>
BobLib follows the VehicleInterfaces subsystem split at the vehicle level:
</p>
<ul>
<li>Chassis and brakes exchange wheel-hub torque through four wheel hubs.</li>
<li>Driveline and transmission exchange rotational power through mechanical flanges.</li>
<li>Battery, inverter, and motor exchange electrical and rotational power through physical connectors.</li>
<li>Driver intent, measured plant information, and actuator commands use the shared <code>controlBus</code>.</li>
<li>Atmosphere-owned ambient signals use <code>AtmosphereBus</code>.</li>
</ul>
<p>
When replacing a subsystem, keep the same public connector contract. A new
chassis should satisfy
<a href='modelica://VehicleInterfaces.Chassis.Interfaces.TwoAxleBase'>VehicleInterfaces.Chassis.Interfaces.TwoAxleBase</a>.
A new driveline should satisfy
<a href='modelica://VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase'>VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase</a>.
A new electric drive should satisfy
<a href='modelica://VehicleInterfaces.ElectricDrives.Interfaces.Base'>VehicleInterfaces.ElectricDrives.Interfaces.Base</a>.
The vehicle template should not need raw signal connectors just because a
subsystem changed.
</p>
<h4>Creating a Vehicle Variant</h4>
<p>
The usual pattern is to extend an existing standard template and redeclare only
the components that change:
</p>
<pre><code>within MyTeam;
model MyVehicleSim

  extends BobLib.Experiments.Standards.VehicleSim(
    redeclare MyTeam.Chassis.MyChassis chassis,
    redeclare MyTeam.Controllers.MyStandardVCU vcu,
    redeclare MyTeam.Brakes.MyBrakes brakes,
    pVehicle = MyTeam.Records.MyVehicleRecord());
end MyVehicleSim;</code></pre>
<p>
If the topology changes substantially, create a new template under
<a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle'>Templates.Vehicle</a>
and make a public entry point under
<a href='modelica://BobLib.Experiments.Standards'>Experiments.Standards</a>.
Keep the top-level public name stable and put detailed combinations in the
template package.
</p>
<h4>Records and Parameters</h4>
<p>
Complete vehicle definitions live under
<a href='modelica://BobLib.Records.VehicleDefn'>Records.VehicleDefn</a>.
They aggregate subsystem-owned records from
<a href='modelica://BobLib.Records.VehicleRecord'>Records.VehicleRecord</a>,
such as <code>pBattery</code>, <code>pVCU</code>, <code>pInverter</code>,
<code>pMotor</code>, <code>pDriveline</code>, and chassis or aero records.
</p>
<p>
Do not reintroduce a monolithic powertrain record as a second parameter path.
If a parameter belongs to a subsystem, put it in that subsystem record and pass
it to the subsystem from the vehicle definition. If a quantity may later vary
with time or become a sensor signal, prefer publishing it from the owning
subsystem on a bus while keeping the current model parameter-backed internally.
</p>
<h4>Replacing Common Subsystems</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Subsystem</th><th>Keep this contract</th><th>Notes</th></tr>
<tr><td>VCU</td><td><a href='modelica://VehicleInterfaces.Controllers.Interfaces.Base'>VehicleInterfaces controller base</a> or the standard BobLib VCU interface required by the template.</td><td>Subscribe to driver and plant buses. Publish actuator requests to motor, driveline, and brake control buses.</td></tr>
<tr><td>Chassis</td><td><a href='modelica://VehicleInterfaces.Chassis.Interfaces.TwoAxleBase'>TwoAxleBase</a>.</td><td>Publish chassis-owned measurements such as speed, ride heights, tire normal loads, and lateral acceleration on <code>chassisBus</code>.</td></tr>
<tr><td>Brakes</td><td><a href='modelica://VehicleInterfaces.Brakes.Interfaces.TwoAxleBase'>brakes two-axle interface</a>.</td><td>Consume <code>brakesControlBus.mechanicalBrakeTorqueRequest</code>. Apply wheel torque opposing wheel angular velocity.</td></tr>
<tr><td>Motor</td><td><a href='modelica://VehicleInterfaces.ElectricDrives.Interfaces.Base'>electric-drive base</a>.</td><td>Consume commands from <code>electricMotorControlBus</code> and publish speed or diagnostics on <code>electricMotorBus</code>.</td></tr>
<tr><td>Driveline</td><td><a href='modelica://VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase'>driveline two-axle interface</a>.</td><td>Keep wheel hubs and transmission flange compatible with the vehicle stack.</td></tr>
<tr><td>Aero</td><td><a href='modelica://BobLib.Aero.Interfaces.Base'>BobLib aero base</a>.</td><td>Subscribe to chassis ride heights and atmosphere signals; apply force and torque to the sprung chassis frame.</td></tr>
<tr><td>Atmosphere</td><td><a href='modelica://VehicleInterfaces.Atmospheres.Interfaces.Base'>VehicleInterfaces atmosphere base</a> plus <a href='modelica://BobLib.Atmospheres.Interfaces.AtmosphereBus'>AtmosphereBus</a>.</td><td>Preserve VI functions and publish ambient signals for subscribers.</td></tr>
</table>
<h4>Driver Environment</h4>
<p>
At the base driver level, the exposed driver signals are steering wheel angle,
accelerator pedal, and brake pedal. The EV vehicle stack can set internal
ready-to-drive or inverter-enable behavior, but EV-specific integer commands
should not leak into the minimal base driver boundary. More specialized
automatic or EV drive-by-wire adapters can add those details explicitly.
</p>
<h4>Construction Checklist</h4>
<ul>
<li>Use the existing VehicleInterfaces connector when one already represents the physical boundary.</li>
<li>Connect every major subsystem to the shared <code>controlBus</code> unless it truly has no signal interaction.</li>
<li>Do not add raw Real inputs for information already published on a bus.</li>
<li>Keep assembly, control law, component physics, and records in separate packages.</li>
<li>After a topology change, run translation and initialization checks before comparing simulation outputs.</li>
</ul>
</html>"));
end ModifyingVehicleConstruction;
