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
<h4>Public structure</h4>
<p>
Use <code>Chassis</code> for suspension, tire, body, and chassis models;
<code>Drivelines</code> for final-drive, differential, and halfshaft
assemblies; <code>EnergyStorage</code> for battery systems;
<code>ElectricDrives</code> for motor-side models; <code>PowerElectronics</code>
for inverter models; and <code>Controllers</code> for VCU/controller models.
Those packages are the intended public entry points.
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
power electronics, electric drive, driveline, VI brakes, driver environment,
road, atmosphere, and world into one replaceable stack.
</p>
<p>
The aero model receives ride heights from the chassis and atmospheric
conditions from the VehicleInterfaces atmosphere functions. The vehicle template
evaluates wind velocity and density at the chassis CG, computes relative
airspeed in the body frame, and connects those signals into the aero subsystem.
</p>
<p>
Simulation templates use <code>headless=false</code> by default so examples open
with MultiBody animation geometry visible. Set <code>headless=true</code> for
batch runs where visualization geometry is not needed.
</p>
</html>"));
end UsersGuide;
