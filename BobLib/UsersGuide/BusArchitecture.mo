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
<tr><td><code>controlBus.driverBus</code></td><td><a href='modelica://BobLib.DriverEnvironments.CommandedDriveByWire'>CommandedDriveByWire</a> or <a href='modelica://BobLib.DriverEnvironments.ExternalDriver'>ExternalDriver</a></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td>Steering wheel angle, accelerator pedal, brake pedal, inverter enable.</td></tr>
<tr><td><code>controlBus.chassisBus</code></td><td><a href='modelica://BobLib.Chassis.BaseClasses.PartialChassis'>PartialChassis</a></td><td><a href='modelica://BobLib.Aero.Interfaces.Base'>Aero</a>; ExternalDriver forwards measurements to StandardDriver</td><td>Vehicle speed, per-corner ride heights, tire normal loads, spring lengths, lateral acceleration.</td></tr>
<tr><td><code>controlBus.batteryBus</code></td><td>Battery pack</td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a>, power electronics models</td><td>HV bus voltage and current.</td></tr>
<tr><td><code>controlBus.electricMotorBus</code></td><td><a href='modelica://BobLib.ElectricDrives.Motor'>Motor</a></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td>Motor shaft speed and motor-side diagnostics.</td></tr>
<tr><td><code>controlBus.electricMotorControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a> (requests); inverter (availablePower)</td><td><a href='modelica://BobLib.ElectricDrives.Motor'>Motor</a>, inverter models</td><td>Power request, limited torque command, VCU active state, regenerative torque limit.</td></tr>
<tr><td><code>controlBus.drivelineControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td>External diagnostics</td><td>Requested rear-axle drive torque. The physical driveline receives transmission shaft torque; this field does not drive a second actuator.</td></tr>
<tr><td><code>controlBus.brakesControlBus</code></td><td><a href='modelica://BobLib.Controllers.VCU'>VCU</a></td><td><a href='modelica://BobLib.Brakes.BasicVCUBrakes'>BasicVCUBrakes</a></td><td>Mechanical brake torque request.</td></tr>
</table>
<h4>Typed subsystem buses</h4>
<p>Each subsystem owns its actual expandable bus type in its <code>Interfaces</code>
package. For example, <a href='modelica://BobLib.Chassis.Interfaces.ChassisBus'>ChassisBus</a>
declares chassis measurements, <a href='modelica://BobLib.Chassis.Suspension.Interfaces.AxleBus'>AxleBus</a>
declares local left/right axle measurements, and
<a href='modelica://BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus'>ElectricMotorControlBus</a>
declares electric-drive commands. These types extend the corresponding VI bus and
are instantiated by the local signal publishers and subscribers.</p>
<p>Before adding a signal, declare its type, units and description in the owning
subsystem bus. Then connect its producer or consumer to that typed local bus.
Connect the local bus to the matching field of the BobLib controlBus.
Do not duplicate the contract in the user guide or change the installed VI library.</p>
<p>OMEdit reads the local bus instance declarations for its member menu.
BobLib-owned umbrella ports use
<a href='modelica://BobLib.Utilities.Interfaces.ControlBus'>Utilities.Interfaces.ControlBus</a>:
expand a subsystem such as chassisBus to browse its declared signals. Subsystem interface bases declare this same typed umbrella, so their external
controlBus ports expose the same member names. Unused declared members remain potentially present, not new sources.</p>
<p>Declare each signal in its owning typed bus before adding a connection. Check the containing model in the Modelica compiler to verify connection types and balance. Inspect the rendered diagram separately.</p>
<p>Axle measurements are flattened at the chassis boundary for OpenModelica compatibility:
<code>Fz_1</code> through <code>Fz_4</code> are the front-left, front-right,
rear-left and rear-right normal loads. Spring lengths use
<code>frontLeftSpringLength</code>, <code>frontRightSpringLength</code>,
<code>rearLeftSpringLength</code> and <code>rearRightSpringLength</code>.
The axle buses remain the signal sources; PartialChassis connects them through local signal taps.</p>
<h4>Publisher Pattern</h4>
<p>
The standard chassis measurement is
<code>controlBus.chassisBus.longitudinalVelocity</code> in m/s: signed velocity
along the chassis forward axis. BobLib additionally publishes
<code>vehicleSpeed</code>, the nonnegative magnitude of all three body velocity
components. These signals differ during reversing and lateral motion and must
not be substituted for one another. Corner suffixes 1, 2, 3, 4 mean front-left,
front-right, rear-left, rear-right, respectively.
</p>
<p>
Publish a subsystem-owned signal by connecting its signal connector to the
proper bus field. Use a source block only when it computes or generates a signal. For example,
<a href='modelica://BobLib.Chassis.BaseClasses.PartialChassis'>PartialChassis</a> computes
<code>rideHeight_1</code> internally and connects that output directly to
<code>controlBus.chassisBus.rideHeight_1</code>.
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
<h4>OMEdit 1.27.1 Connection Chooser</h4>
<p>OMEdit lists inherited expandable-bus members but disables their selection.
This follows the inherited-item filter in its
<a href='https://github.com/OpenModelica/OpenModelica/blob/v1.27.1/OMEdit/OMEditLIB/Annotations/LineAnnotation.cpp'>connection chooser implementation</a>;
grey entries do not mean that the BobLib contract is missing.</p>
<p>When a member cannot be selected graphically, use Text View to write the
qualified connection, for example
<code>connect(speedSensor.w, controlBus.electricMotorBus.speed);</code>,
then add its diagram route and check the model. Declare any new signal in the
owning subsystem bus first. Use existing local typed bus taps where available;
do not duplicate inherited ports or create a second publisher to make an editor
menu selectable.</p>
<h4>Ownership Rules</h4>
<p>
Every BobLib subsystem exposes <code>BobLib.Utilities.Interfaces.ControlBus</code>.
Domain interface bases preserve the VI physical connectors while owning this typed
port, because the upstream port is not replaceable. All umbrella members use
BobLib contracts; unused subsystem buses are omitted. Local subsystem taps
use the same subsystem-owned types.
Upstream <code>Internal.StandardBus</code> definitions remain reference-only;
BobLib actual bus contracts live under each domain's <code>Interfaces</code> package. Keep physical measurements on the
subsystem bus and controller commands on its corresponding control bus.
</p>
<ul>
<li>The chassis owns chassis motion, tire loads, ride heights, and chassis-frame accelerations.</li>
<li>The atmosphere owns wind, density, temperature, humidity, and pressure.</li>
<li>The driver generates intent; the driver environment maps and publishes it on driverBus.</li>
<li>The VCU owns actuator requests, command limits, and controller status.</li>
<li>The motor owns motor measurements. The battery owns battery measurements.</li>
</ul>
<p>
If a signal's owner is unclear, choose the subsystem that would still know the
value if every other subsystem were replaced by a higher- or lower-fidelity
model.
</p>
<h4>Replacement Requirements</h4>
<p>The generic <code>PartialVehicle</code> boundary uses VI subsystem contracts.
<code>PartialElectricVehicle</code> adds the detailed BobLib chassis and optional
differential-brake contracts. PartialVehicleFMI requires the commanded
steering/pedal environment; PartialVehicleSim uses the VI external-driver path. A stock VI
replacement must supply any additional signals its connected controller consumes.</p>
<p>The detailed electric vehicle also requires the motor's
<code>chassisFrame</code> attachment. Stock VI electric drives expose only the
shaft bearing frame. An adapter must add a chassis frame and an internal
<code>VehicleInterfaces.PowertrainMounts.SingleSystemRigidMount</code>, connecting
its mount to <code>shaft_b.bearingFrame</code>. Use the motor calibration's mount
offset so housing reactions act at the physical mount. A bare stock motor is
therefore not a direct replacement for this extended contract.</p>
<ul>
<li><code>electricMotorBus.speed</code>: shaft speed in rad/s. BobLib Motor publishes it; a VI SimpleMotorDC adapter can measure its shaft angle derivative.</li>
<li><code>batteryBus.voltage</code> and <code>batteryBus.current</code>: terminal voltage and discharge-positive current. For VI Battery these are pin_p.v-pin_n.v and -pin_p.i.</li>
<li><code>electricMotorControlBus.availablePower</code>: inverter-supplied power capability consumed by BobLib's averaged motor. Measurement compatibility alone does not make a stock DC machine implement this command law.</li>
<li><code>transmissionControlBus.currentGear</code>: FixedRatioTransmission publishes 1 for its permanently engaged reducer.</li>
<li><code>driverBus.requestedGear</code>: AutomaticDriveByWire publishes manualGearRequest; gearboxMode determines whether direct selection applies.</li>
</ul>
<p>Publish each field once. Adapter measurements supplement a component that lacks them; do not connect them alongside a component that already publishes the same field. Electrical equation balance is a necessary check, not proof of equivalent torque-control behavior.</p>
<h4>Parameter-Backed Signals</h4>
<p>
Bus-owned actuator capabilities may depend on numerical calibration, operating
conditions, or measured limits. The inverter publishes availablePower and the
motor consumes it through electricMotorControlBus. Publish that field once;
keep the VCU's requested power distinct from the inverter's available power.
Atmospheric fields use VI function queries instead of this bus pattern.

</p>
</html>"));
end BusArchitecture;
