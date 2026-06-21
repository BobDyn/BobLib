within BobLib.UsersGuide;
class Overview

  "Orientation for the BobLib package"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>What BobLib Is</h4>
<p>
BobLib is the low-level Modelica vehicle modeling layer for BobDyn. It contains
physical vehicle models, parameter records, standard simulation templates, and
regression fixtures for detailed FSAE-style vehicle dynamics development.
</p>
<p>
The library is organized around
<a href='modelica://VehicleInterfaces'>VehicleInterfaces</a> 2.0.2. Public
subsystem models expose the standard VehicleInterfaces connector surface first,
then place BobLib physics and implementation details one level deeper. This
keeps vehicle-level assembly clean while still allowing high-fidelity internal
models.
</p>
<h4>Core Idea</h4>
<p>
Subsystems own and publish the information they create. Other subsystems
subscribe when they need that information. Physical power, force, torque, frame,
electrical, and fluid exchange still use physical connectors. Informational and
command signals use the shared bus namespace.
</p>
<p>
For example, the chassis publishes vehicle speed, ride heights, tire normal
loads, and lateral acceleration on <code>controlBus.chassisBus</code>. The aero
model subscribes to ride heights, and the standard VCU subscribes to speed,

lateral acceleration, and normal loads. The atmosphere publishes density,
temperature, pressure, humidity, and wind on an
<a href='modelica://BobLib.Atmospheres.Interfaces.AtmosphereBus'>AtmosphereBus</a>
so the aero model can calculate relative airspeed without raw atmosphere inputs.
</p>
<h4>Public Package Map</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Package</th><th>Use</th></tr>
<tr><td><a href='modelica://BobLib.Experiments'>Experiments</a></td><td>Runnable standard entry points and reusable vehicle, FMI, and four-post templates.</td></tr>
<tr><td><a href='modelica://BobLib.Records'>Records</a></td><td>Vehicle records and subsystem-owned parameter records.</td></tr>
<tr><td><a href='modelica://BobLib.Controllers'>Controllers</a></td><td>VCU models and standard maneuver generation.</td></tr>
<tr><td><a href='modelica://BobLib.DriverEnvironments'>DriverEnvironments</a></td><td>Driver intent adapters that publish steering, accelerator, and brake pedal signals.</td></tr>
<tr><td><a href='modelica://BobLib.Chassis'>Chassis</a></td><td>Body, suspension, tires, brakes, and chassis VehicleInterfaces adapters.</td></tr>
<tr><td><a href='modelica://BobLib.Drivelines'>Drivelines</a></td><td>Final drive, differential, and halfshaft assemblies.</td></tr>
<tr><td><a href='modelica://BobLib.ElectricDrives'>ElectricDrives</a></td><td>Motor-side models and motor implementation cores.</td></tr>
<tr><td><a href='modelica://BobLib.EnergyStorage'>EnergyStorage</a></td><td>Battery systems and HV storage models.</td></tr>
<tr><td><a href='modelica://BobLib.PowerElectronics'>PowerElectronics</a></td><td>Inverter and DC-side power electronics.</td></tr>
<tr><td><a href='modelica://BobLib.Transmissions'>Transmissions</a></td><td>Fixed-ratio and future gearbox or clutch models.</td></tr>
<tr><td><a href='modelica://BobLib.Atmospheres'>Atmospheres</a></td><td>Ambient-condition models that preserve VehicleInterfaces functions and publish atmosphere signals.</td></tr>
<tr><td><a href='modelica://BobLib.Aero'>Aero</a></td><td>Aerodynamic load models that subscribe to chassis and atmosphere signals.</td></tr>
<tr><td><a href='modelica://BobLib.Utilities'>Utilities</a></td><td>Shared math and mechanics helpers, especially reusable MultiBody fixtures and functions.</td></tr>
</table>
<h4>Where Implementation Belongs</h4>
<p>
The first level of each public domain package is the shared adapter or concrete
insertion point used by experiments. BobLib physics, implementation details,
templates, and helper models live one level deeper in packages such as
<code>Internal</code>, <code>Templates</code>, <code>Mounts</code>,
<code>Suspension</code>, <code>Brakes</code>, or
<code>Suspension.Tires</code>. Avoid duplicate public access paths for the same
interface or model.
</p>
<p>
Tests live outside the production package in the sibling Modelica package
<code>Tests/BobLibTest</code>. Production packages should not contain
regression or component test models.
</p>
<h4>Recommended Reading Path</h4>
<ol>
<li>Run one of the standard templates using
<a href='modelica://BobLib.UsersGuide.RunningExistingTemplates'>Running Existing Templates</a>.</li>
<li>Read
<a href='modelica://BobLib.UsersGuide.BusArchitecture'>Bus Architecture</a>
before adding raw signal inputs or outputs.</li>
<li>Use
<a href='modelica://BobLib.UsersGuide.ModifyingVehicleConstruction'>Modifying Vehicle Construction</a>
when replacing subsystems at the full-vehicle level.</li>
<li>Use
<a href='modelica://BobLib.UsersGuide.ModifyingLumpedModels'>Modifying Lumped Models</a>
for black-box or low-order component models.</li>
<li>Use
<a href='modelica://BobLib.UsersGuide.ModifyingCorePhysics'>Modifying Core Physics</a>
for suspension, tire, contact, and MultiBody changes.</li>
<li>Use
<a href='modelica://BobLib.UsersGuide.ValidationAndTesting'>Validation and Testing</a>
before trusting a modified branch.</li>
</ol>
<h4>Supported Stack</h4>
<p>
Current checks target Modelica Standard Library 4.1.0,
VehicleInterfaces 2.0.2, and OpenModelica. The included templates are
regression-tested baselines, not validated models of any specific vehicle.
Validate records and subsystems against measured data before making design
decisions.
</p>
</html>"));
end Overview;
