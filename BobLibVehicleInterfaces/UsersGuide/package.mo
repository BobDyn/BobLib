within BobLibVehicleInterfaces;
package UsersGuide
  "User guide for BobLibVehicleInterfaces"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Purpose</h4>
<p>
This package is a standalone implementation of BobLib models organized as if
the VehicleInterfaces 2.0.2 contract were the starting point. Public models
should extend VehicleInterfaces subsystem interfaces first, then place BobLib
physics and data beneath those boundaries.
</p>
<p>
The package is a model-development layer, not a complete workflow layer. Use
BobSim for repeatable study execution, signal extraction, metrics, plots,
reports, envelope maps, and sensitivity studies. Use this package directly when
you are developing, replacing, or inspecting the underlying Modelica subsystem
models.
</p>
<h4>Public structure</h4>
<p>
Use <code>Chassis</code> for suspension, tire, body, and chassis models;
<code>Drivelines</code> for final-drive, differential, and halfshaft
assemblies; <code>EnergyStorage</code> for battery systems;
<code>ElectricDrives</code> for motor-side models; <code>Engines</code> for
internal-combustion reference and future engine models;
<code>Transmissions</code> for fixed-ratio, gearbox, clutch, and launch-device
models; <code>DriverEnvironments</code> for driver and EV command adapters;
<code>PowerElectronics</code> for inverter models; <code>Atmospheres</code> for
ambient-condition models; and <code>Controllers</code> for VCU/controller
models. Those packages are the intended public entry points.
</p>
<p>
Within each public subsystem domain, models directly under the package are the
models intended to extend or adapt the vehicle-level VehicleInterfaces stack.
They are the shared-contract layer; some are partial adapters, and concrete
variants bind records and implementation choices for experiments. Reusable
BobLib physics and implementation pieces begin one package level deeper in
nested packages such as
<code>Internal</code>, <code>Mounts</code>, <code>Suspension</code>,
<code>Suspension.Tires</code>, or physics-specific <code>Templates</code>.
</p>
<h4>Current boundary</h4>
<p>
VehicleInterfaces defines a canonical chassis/driveline/brakes split using four
<code>FlangeWithBearing</code> wheel hubs and expandable control buses. The
primary <code>Experiments.Standards.VehicleSim</code> follows that topology and
explicitly redeclares the detailed BobLib chassis, energy storage, controller,
power electronics, electric drive, transmission, driveline, VI brakes, driver
commands, road, atmosphere, and world into one replaceable stack.
</p>
<p>
Standard full-vehicle simulations are autonomous by default: the vehicle
template owns maneuver-intent sources, but those sources cross a
driver-environment adapter before entering the shared
<code>driverBus</code>. Subsystems broadcast the information they own:
the chassis publishes ride heights and speed on <code>chassisBus</code>, the
battery publishes terminal measurements on <code>batteryBus</code>, and the
motor publishes shaft measurements on <code>electricMotorBus</code>. The VCU
subscribes to those driver and plant-measurement buses, then publishes actuator
requests on the downstream control buses. Positive speed-control torque is sent
to the electric-drive path, while negative speed-control torque is split by the
regen blend and the remaining mechanical-brake request is published on
<code>brakesControlBus</code>. The default regen blend is zero, so speed-control
braking demand goes to the mechanical brake model. BobLib atmosphere models
preserve the VehicleInterfaces function contract while
publishing density, wind velocity, temperature, humidity, and pressure on a
shared <code>AtmosphereBus</code>. Those values may be parameter-backed in a
constant atmosphere today, but they are still published as signals so later
weather, altitude, or noise models can vary them without changing subscribers.
</p>
<p>
Reusable redeclare stacks for standard experiments live in
<code>Experiments.Standards.Templates</code>. Full-vehicle EV
battery-inverter-motor-differential simulations live under
<code>Templates.Vehicle</code>; four-post suspension fixture simulations live
under <code>Templates.FourPost</code> and remain PTN-agnostic at the model-name
level.
</p>
<p>
The chassis publishes per-corner ride-height measurements onto
<code>controlBus.chassisBus</code>, and aero models subscribe to those chassis
signals through their own control-bus tap. The atmosphere publishes ambient
signals onto <code>AtmosphereBus</code>, and aero models subscribe to that bus
to combine wind with their sprung chassis frame velocity when computing
relative airspeed. The inverter subscribes to
<code>controlBus.electricMotorControlBus.powerRequest</code> rather than
receiving a direct VCU connector.
</p>
<p>
Simulation templates use <code>headless=false</code> by default so examples open
with MultiBody animation geometry visible. Set <code>headless=true</code> for
batch runs where visualization geometry is not needed.
</p>
<h4>Validation expectation</h4>
<p>
Treat the included experiments as regression-tested baselines, not as validated
models of any specific vehicle. Before using outputs for design decisions,
validate the relevant records and subsystems against measured corner weights,
inertias, suspension kinematics, damper curves, tire data, aero maps,
drivetrain limits, controller behavior, and track-test signals. Keep
<code>make ci</code> passing as records or subsystem models are adapted.
</p>
</html>"));
end UsersGuide;
