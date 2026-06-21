within BobLib.UsersGuide;
class BusArchitecture

  "Publish and subscribe signal architecture"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Purpose</h4>
<p>
The shared buses are the informational namespace between subsystems. A
subsystem broadcasts the information it owns. Other subsystems subscribe to the
fields they need. This keeps vehicle-level diagrams clean and makes it possible
to raise or lower model fidelity without changing the whole vehicle assembly.
</p>
<p>
The bus is not a replacement for physical connectors. Forces, torques,
electrical power, rotational power, frames, and wheel hubs should remain
physical connections. The bus is for driver intent, measured plant signals,
actuator commands, ambient conditions, limits, and diagnostics that may be
time-varying.
</p>
<h4>When to Use a Bus</h4>
<ul>
<li>Use a bus when a signal is owned by one subsystem and may be consumed by several others.</li>
<li>Use a bus when a parameter-backed value might later become time-varying, noisy, measured, or environment-driven.</li>
<li>Use a bus for commands from controllers to actuators.</li>
<li>Use a bus for subsystem measurements, limits, and status flags.</li>
<li>Do not add a raw Real input when the same information already exists on a shared bus.</li>
<li>Do not use a bus to move conserved physical quantities that already have a physical connector.</li>
</ul>
<h4>Current Bus Map</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Bus</th><th>Primary publisher</th><th>Typical subscribers</th><th>Signals</th></tr>
<tr><td><code>controlBus.driverBus</code></td><td><a href='modelica://BobLib.DriverEnvironments.Internal.Driver'>Driver</a></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td>Steering wheel angle, accelerator pedal, brake pedal, inverter enable.</td></tr>
<tr><td><code>controlBus.chassisBus</code></td><td><a href='modelica://BobLib.Chassis.ChassisBase'>ChassisBase</a></td><td><a href='modelica://BobLib.Controllers.StandardVCU'>StandardVCU</a>, <a href='modelica://BobLib.Aero.Interfaces.Base'>Aero</a></td><td>Vehicle speed, per-corner ride heights, tire normal loads, lateral acceleration.</td></tr>
<tr><td><code>controlBus.batteryBus</code></td><td>Battery pack</td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a>, power electronics models</td><td>HV bus voltage and current.</td></tr>
<tr><td><code>controlBus.electricMotorBus</code></td><td><a href='modelica://BobLib.ElectricDrives.Motor'>Motor</a></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td>Motor shaft speed and motor-side diagnostics.</td></tr>
<tr><td><code>controlBus.electricMotorControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td><a href='modelica://BobLib.ElectricDrives.Motor'>Motor</a>, inverter models</td><td>Power request, limited torque command, VCU active state, regenerative torque limit.</td></tr>
<tr><td><code>controlBus.drivelineControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td><a href='modelica://BobLib.Drivelines.RearFinalDriveDifferential'>RearFinalDriveDifferential</a></td><td>Rear-axle drive torque command.</td></tr>
<tr><td><code>controlBus.brakesControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td><a href='modelica://BobLib.Chassis.Brakes.BasicVCUBrakes'>BasicVCUBrakes</a></td><td>Mechanical brake torque request.</td></tr>
<tr><td><a href='modelica://BobLib.Atmospheres.Interfaces.AtmosphereBus'>AtmosphereBus</a></td><td><a href='modelica://BobLib.Atmospheres.ConstantAtmosphere'>ConstantAtmosphere</a> or another atmosphere model</td><td><a href='modelica://BobLib.Aero.Interfaces.Base'>Aero</a></td><td>Density, temperature, pressure, humidity, and world-frame wind velocity.</td></tr>
</table>
<h4>Publisher Pattern</h4>
<p>
Publish a subsystem-owned signal with a small source block connected to the
proper bus field. For example,
<a href='modelica://BobLib.Chassis.ChassisBase'>ChassisBase</a> computes
<code>rideHeight_1</code> internally and connects a
<code>RealExpression</code> to <code>controlBus.chassisBus.rideHeight_1</code>.
The vehicle model only connects the subsystem to <code>controlBus</code>; it
does not route the ride-height signal directly.
</p>
<h4>Subscriber Pattern</h4>
<p>
Subscribe with a local typed tap and assign it to an internal variable. For
example,
<a href='modelica://BobLib.Aero.Interfaces.Base'>Aero.Interfaces.Base</a>
connects <code>controlBus.chassisBus.rideHeight_1</code> to a local
<code>RealInput</code>, then uses that internal value in the aero equations.
This makes the subscription visible in the diagram without adding a raw public
input to the vehicle-level component.
</p>
<h4>Ownership Rules</h4>
<ul>
<li>The chassis owns chassis motion, tire loads, ride heights, and chassis-frame accelerations.</li>
<li>The atmosphere owns wind, density, temperature, humidity, and pressure.</li>
<li>The driver environment owns driver intent.</li>
<li>The VCU owns actuator requests, command limits, and controller status.</li>
<li>The motor owns motor measurements. The battery owns battery measurements.</li>
</ul>
<p>
If a signal's owner is unclear, choose the subsystem that would still know the
value if every other subsystem were replaced by a higher- or lower-fidelity
model.
</p>
<h4>Parameter-Backed Signals</h4>
<p>
Some bus fields are currently backed by parameters. That is intentional when
the quantity may later vary. A constant atmosphere can publish pressure and
temperature from parameters today, while a weather, altitude, or sensor-noise
model can publish time-varying signals tomorrow. Subscribers do not change.

</p>
</html>"));
end BusArchitecture;
