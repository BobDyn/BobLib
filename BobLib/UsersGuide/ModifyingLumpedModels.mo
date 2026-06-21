within BobLib.UsersGuide;
class ModifyingLumpedModels

  "How to build compatible lumped subsystem models"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Adapter Plus Core Pattern</h4>
<p>
Lumped models should separate the public adapter from the implementation core.
The public model sits at the package's VehicleInterfaces boundary and exposes
the standard connector surface. Detailed equations, maps, filters, limiters,
and helper components live one level deeper in <code>Internal</code>,
<code>Templates</code>, or a purpose-specific nested package.
</p>
<p>
This lets a team replace a low-order model with a high-fidelity one without
rewiring the vehicle. The vehicle should still see the same physical connectors
and the same bus fields.
</p>
<h4>Connector Rules</h4>
<ul>
<li>Use physical connectors for conserved exchange: flange torque and speed, wheel hubs, frames, pins, and hydraulic or mechanical ports.</li>
<li>Use the control bus for informational signals, commands, limits, status flags, and measurements.</li>
<li>Do not expose raw Real inputs for bus-owned data such as ride heights, vehicle speed, brake request, or motor torque request.</li>
<li>Do not publish another subsystem's signal unless your model is explicitly transforming or estimating it.</li>
<li>Keep units and sign conventions explicit on public connectors and bus taps.</li>
</ul>
<h4>Motor and Inverter Models</h4>
<p>
The public motor model
<a href='modelica://BobLib.ElectricDrives.Motor'>ElectricDrives.Motor</a>
extends the VehicleInterfaces electric-drive contract. Its implementation core
is
<a href='modelica://BobLib.ElectricDrives.Internal.PowerLimitedMotor'>PowerLimitedMotor</a>.
The motor subscribes to commands on <code>electricMotorControlBus</code> and
publishes motor-owned measurements on <code>electricMotorBus</code>.
</p>
<p>
A replacement motor can be a simple torque source, a power-limited map, a
thermal derating model, or an efficiency-map model. It should still accept the
same bus commands and physical electrical or rotational connectors expected by
the vehicle stack. If a new limit may be useful to the VCU, add it as a
published signal or a clearly owned command field rather than a private raw
wire.
</p>
<h4>VCU Models</h4>
<p>
The base
<a href='modelica://BobLib.Controllers.VCU'>VCU</a> subscribes to driver,
chassis, battery, and motor buses. It publishes actuator requests to
<code>electricMotorControlBus</code>, <code>drivelineControlBus</code>, and
<code>brakesControlBus</code>. The standard maneuver generator
<a href='modelica://BobLib.Controllers.StandardVCU'>StandardVCU</a> extends
that base VCU and adds autonomous steering, pedal, inverter-enable, ramp-steer,
step-steer, sine-steer, and steady-state lateral-acceleration modes.
</p>
<p>
Positive speed-control torque is sent to the electric-drive path. Negative
speed-control torque is split by the regenerative blend and the remaining
mechanical-brake request is published to the brake bus. The current default
regen blend is zero, so speed-control braking demand goes to the mechanical
brakes.
</p>
<h4>Brake Models</h4>
<p>
<a href='modelica://BobLib.Chassis.Brakes.BasicVCUBrakes'>BasicVCUBrakes</a>
is a simple VCU-commanded brake model. It consumes
<code>brakesControlBus.mechanicalBrakeTorqueRequest</code>, splits torque by
mechanical brake bias, and applies torque opposite wheel angular velocity.
Future pedal-box, hydraulic compliance, bias-bar motion, damping, caliper, and
pad models should keep the same vehicle-level contract unless the physical
brake interface itself changes.
</p>
<h4>Driveline and Differential Models</h4>
<p>
<a href='modelica://BobLib.Drivelines.RearFinalDriveDifferential'>RearFinalDriveDifferential</a>
is the public driveline adapter. Differential implementation models such as
<a href='modelica://BobLib.Drivelines.Internal.Differential1D'>Differential1D</a>
and
<a href='modelica://BobLib.Drivelines.Internal.LockedDifferential1D'>LockedDifferential1D</a>
belong in <code>Drivelines.Internal</code>. Keep final-drive ratio,
halfshaft stiffness and damping, preload, lock fraction, and clutch parameters
in the driveline record.
</p>
<p>
If the driveline needs a command, subscribe to the appropriate driveline
control-bus field. Wheel torques, halfshaft dynamics, and differential
kinematics remain mechanical equations, not bus signals.
</p>
<h4>Aero Models</h4>
<p>
Aerodynamic models should extend
<a href='modelica://BobLib.Aero.Interfaces.Base'>Aero.Interfaces.Base</a>.
That base model already subscribes to chassis ride heights on
<code>controlBus.chassisBus</code>, subscribes to density and wind on
<a href='modelica://BobLib.Atmospheres.Interfaces.AtmosphereBus'>AtmosphereBus</a>,
computes relative airspeed from the sprung chassis frame, and applies force and
torque to the chassis through a rigid aero mount.
</p>
<p>
<a href='modelica://BobLib.Aero.CFDAeroMap'>CFDAeroMap</a> demonstrates the
pattern. It averages per-corner ride heights into front and rear ride heights,
looks up CFD tables, scales by density and relative airspeed, and produces
body-frame force and torque.
</p>
<h4>Atmosphere Models</h4>
<p>
Atmosphere models should preserve the
<a href='modelica://VehicleInterfaces.Atmospheres.Interfaces.Base'>VehicleInterfaces atmosphere function contract</a>
and publish atmosphere-owned signals on
<a href='modelica://BobLib.Atmospheres.Interfaces.AtmosphereBus'>AtmosphereBus</a>.
<a href='modelica://BobLib.Atmospheres.ConstantAtmosphere'>ConstantAtmosphere</a>
uses parameters today, but those same bus fields can later come from altitude,
weather, track position, sensor noise, or a recorded data source.
</p>
<h4>Compatibility Checklist</h4>
<ul>
<li>Start from the public interface model closest to the subsystem you are replacing.</li>
<li>Keep the public connector names, units, and sign conventions stable.</li>
<li>Put implementation equations in a nested package when they are reusable or complex.</li>
<li>Publish only subsystem-owned information.</li>
<li>Subscribe through bus taps rather than adding vehicle-level raw wires.</li>
<li>Add a component or regression fixture under <code>Tests/BobLibTest</code> for the new behavior.</li>
</ul>
</html>"));
end ModifyingLumpedModels;
