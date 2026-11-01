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
or detailed control logic. PartialElectricVehicle owns plant wiring and shared measurements. The maneuver wrapper
<a href='modelica://BobLib.Experiments.Standards.BaseClasses.PartialVehicleSim'>PartialVehicleSim</a>
connects physical ports, connects the shared bus, instantiates road,
atmosphere, aero, and world, and provides standard outputs and termination
monitors. The controller behavior belongs in
<a href='modelica://BobLib.Drivers.StandardDriver'>StandardDriver</a> or another
driver model; actuator limits and blending remain in the VCU.
</p>
<h4>Calibration Ownership</h4>
<p>Each assembly passes one physical parameter record to each BobLib subsystem:
<code>pVehicle.chassis</code> becomes <code>chassis.pChassis</code>, whose
<code>frontAxle</code> and <code>rearAxle</code> records become each axle's
<code>pAxle</code>. Tire, wheel, frame, brake and powertrain records follow the
same ownership pattern. Modify the owning record rather than duplicating values
on its child components. Replaceable model choices, connector switches and
experiment inputs remain separate from physical calibration.</p>
<p><code>VehicleSim.pInitialization</code> is a saved starting pose for the
supplied default vehicle. Recompute it after changing geometry or static loading;
it is not part of the reusable physical vehicle record.</p>
<p>All nine supplied VehicleDefn records link brake input reduction to
<code>driveline.inputRatio</code>, and VCU reduction to
<code>transmission.gearRatio*driveline.inputRatio</code>. Preserve those
relationships when defining a new vehicle. Standalone subsystem assemblies must
supply mutually consistent records themselves.</p>
<h4>Differential Architecture</h4>
<p>Select <code>driveline.differentialArchitecture</code> at model level, or set
<code>pVehicle.driveline.differentialArchitecture</code> in the selected vehicle record.
Choices are Open, ClutchLSD, TorqueBiasing, and Locked from
<code>BobLib.Drivelines.Types.DifferentialArchitecture</code>.</p>
<ul>
<li>Open: equal output torques, no slip resistance.</li>
<li>ClutchLSD: preloadTorque and drive/coastLockingFraction set static transfer
capacity. slidingFrictionRatio is an Advanced setting. Preload is inter-output
transfer torque; output torque difference is twice this value.</li>
<li>TorqueBiasing: driveTorqueBiasRatio and coastTorqueBiasRatio bound the
high/low output torque ratio in same-sign steady transmission. There is no
preload and capacity vanishes at zero input torque.</li>
<li>Locked: outputs share one angle; external halfshaft compliance remains.</li>
</ul>
<p>Halfshaft parameters are per side: one shared <code>pDriveline.halfshaft.stiffness</code>,
<code>pDriveline.halfshaft.damping</code>, and <code>pDriveline.halfshaft.inertia</code> set configures both branches.
The inertia is modeled at each differential output; the default damping is
<code>2*sqrt(stiffness*inertia)</code>. These values are not axle totals.</p>
<p>ClutchLSD represents plate-LSD behavior; TorqueBiasing represents effective
helical-LSD behavior. Defaults are illustrative, not calibrated Drexler or
Quaife specifications. Both use MSL static/sliding friction. Drive/coast selection
uses signed input torque; reverse-direction power-based selection is not modeled.
The four-post central-brake calculation still assumes an open differential.</p>
<h4>VehicleInterfaces Boundary</h4>
<p>
BobLib follows the VehicleInterfaces subsystem split at the vehicle level:
</p>
<ul>
<li>Chassis and brakes exchange wheel-hub torque through four wheel hubs.</li>
<li>Driveline and transmission exchange rotational power through mechanical flanges.</li>
<li>Battery, inverter, and motor exchange electrical and rotational power through physical connectors.</li>
<li>Driver intent, measured plant information, and actuator commands use the shared <code>controlBus</code>.</li>
<li>Aero queries the shared VI atmosphere functions at its load point.</li>
</ul>
<p>
The reference workflows are
<a href='modelica://VehicleInterfaces.UsersGuide.QuickStart'>VehicleInterfaces Quick Start</a>,
<a href='modelica://VehicleInterfaces.Chassis.Tutorial'>the chassis tutorial</a>, and
<a href='modelica://VehicleInterfaces.UsersGuide.SignalBus'>the signal-bus guide</a>.
BobLib targets VehicleInterfaces 2.0.2 with Modelica 4.1.0.
Public brake implementations live in <code>BobLib.Brakes</code>, parallel to
<code>BobLib.Chassis</code>; wheel attachment does not make brakes a nested
chassis subsystem.
</p>
<p>
When replacing a subsystem, keep the same public connector contract. A new
chassis should satisfy
<a href='modelica://BobLib.Chassis.Interfaces.Base'>BobLib.Chassis.Interfaces.Base</a>.
A new driveline should satisfy
<a href='modelica://VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase'>VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase</a>.
A new electric drive should satisfy
<a href='modelica://BobLib.ElectricDrives.Interfaces.Base'>BobLib.ElectricDrives.Interfaces.Base</a>.
Matching physical connectors is necessary but is not sufficient for replacing
a subsystem in the current maneuver templates. Their calibration modifiers,
bus signals, optional connectors, and monitored outputs must also match.
</p>
<h4>Current Replacement Scope</h4>
<p>
<code>PartialVehicleSim</code> is a calibrated BobLib maneuver experiment, not
a generic VehicleInterfaces vehicle assembly. It reads BobLib chassis motion,
tire-load and steering outputs and StandardDriver maneuver diagnostics. Its
subsystem declarations also supply BobLib-specific parameter modifiers.
Use compatible BobLib models in its replacement menus. An unrelated model
that only extends a VehicleInterfaces base requires adapting these modifiers
and monitors before it can replace the supplied component.
</p>
<p>
For a new generic assembly, follow
<a href='modelica://VehicleInterfaces.Examples.PartialVehicle'>VehicleInterfaces.Examples.PartialVehicle</a>:
constrain subsystem choices by the appropriate VehicleInterfaces base, connect
physical ports and the shared control bus, and keep experiment-specific
monitoring separate. Publish standard signal names with their defined units
and meaning; document BobLib additions separately.
</p>
<p>
Optional wheel-bearing and mount connectors must match on both connected
subsystems. These options select supported mechanical interfaces; they do not
select suspension architecture or establish brake reaction paths. A model
must wire every enabled physical connector to the corresponding mechanism.
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
    redeclare MyTeam.Drivers.MyStandardDriver driver,
    redeclare MyTeam.Brakes.MyBrakes brakes,
    redeclare record VehicleRecord = MyTeam.Records.MyVehicleRecord);
end MyVehicleSim;</code></pre>
<p>
If the topology changes substantially, create a new template under
<a href='modelica://BobLib.Experiments.Standards.BaseClasses'>BaseClasses</a>
and make a public entry point under
<a href='modelica://BobLib.Experiments.Standards'>Experiments.Standards</a>.
Keep public entry points stable. Configure independent axle choices in user models; keep only a few illustrative examples in the production library.
</p>
<h4>Sprung Frame Selection</h4>
<p>The chassis parameter dialog exposes <strong>Plant Models / Sprung frame</strong>
independently of the front and rear axles. FrameModel defaults to Body.FrameCompX
(torsional compliance about the longitudinal axis). Select Body.FrameRigid for a
rigid connection, or provide a model extending Body.FrameBase. The chassis supplies
front/rear reference positions and sprung mass properties through the common base;
compliance-specific parameters belong only to the compliant model selection.</p>

<pre><code>extends BobLib.Experiments.Standards.VehicleSim(
  chassis(redeclare model FrameModel = BobLib.Chassis.Body.FrameRigid));</code></pre>
<h4>Records and Parameters</h4>
<p>
For the current double-wishbone family, open the chassis parameter dialog and
use <strong>Plant Models / Front axle</strong> and
<strong>Plant Models / Rear axle</strong> to choose the two implementations
independently. The same axle class works at either end; the vehicle record
supplies its front or rear geometry. Direct, bellcrank, and bellcrank with
stabilizer-bar actuation are implemented. MacPherson and other suspension
families are not yet implemented, and the current chassis configuration still

constrains both choices to the double-wishbone base. For another architecture, extend Chassis.BaseClasses.PartialChassis, redeclare the two AxleBase components and body, and supply contact positions and ride-height offsets. Initialize architecture-specific wheel and suspension states in that specialization. Set steerable=false on fixed-steering axles; lockedRackPosition selects their rack displacement. Publish leftNormalLoad, rightNormalLoad, leftSpringLength, and rightSpringLength on each axleBus; do not reach into another axle implementation for its measurements.
</p>
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
<tr><td>VCU</td><td><a href='modelica://BobLib.Controllers.Interfaces.Base'>VehicleInterfaces controller base</a> or the standard BobLib VCU interface required by the template.</td><td>Subscribe to driver and plant buses. Publish actuator requests to motor, driveline, and brake control buses.</td></tr>
<tr><td>Chassis</td><td><a href='modelica://BobLib.Chassis.BaseClasses.PartialChassis'>BobLib detailed chassis contract</a> for the EV template; the generic assembly uses VI TwoAxleBase.</td><td>Publish chassis-owned measurements such as speed, ride heights, tire normal loads, and lateral acceleration on <code>chassisBus</code>.</td></tr>
<tr><td>Brakes</td><td><a href='modelica://BobLib.Brakes.Interfaces.Base'>BobLib brake contract</a>, extending VI TwoAxleBase with the optional differential-input station.</td><td>Consume <code>brakesControlBus.mechanicalBrakeTorqueRequest</code>. Apply wheel torque opposing wheel angular velocity.</td></tr>
<tr><td>Motor</td><td><a href='modelica://BobLib.ElectricDrives.Interfaces.Base'>BobLib electric-drive contract</a>, preserving VI electrical and shaft connectors and adding the chassis attachment for an internal mount.</td><td>Consume commands from <code>electricMotorControlBus</code> and publish speed or diagnostics on <code>electricMotorBus</code>. Stock VI motors need a mount and measurement adapter.</td></tr>
<tr><td>Driveline</td><td><a href='modelica://BobLib.Drivelines.Interfaces.Base'>BobLib driveline contract</a>, extending VI TwoAxleBase with the optional differential brake shaft.</td><td>Keep wheel hubs and transmission flange compatible with the vehicle stack.</td></tr>
<tr><td>Aero</td><td><a href='modelica://BobLib.Aero.Interfaces.Base'>BobLib aero base</a>.</td><td>Subscribe to chassis ride heights, query VI atmosphere functions, and apply force and torque to the sprung chassis frame.</td></tr>
<tr><td>Atmosphere</td><td><a href='modelica://VehicleInterfaces.Atmospheres.Interfaces.Base'>VehicleInterfaces atmosphere base</a>.</td><td>Implement VI position-dependent atmosphere functions.</td></tr>
</table>
<h4>Brake Placement</h4>
<p>Set <code>frontBrakePlacement</code> and <code>rearBrakePlacement</code> on the experiment. Choices from <a href='modelica://BobLib.Brakes.Types.Placement'>Brakes.Types.Placement</a> are <code>Outboard</code> (upright reaction), <code>Inboard</code> (one brake per halfshaft with sprung-frame reaction), and <code>DifferentialInput</code> (one brake upstream of the differential). Outboard is the default. The rear-drive vehicle supports DifferentialInput at the rear only; the four-post rig supports it on either axle.</p>
<p>Suspension axles and chassis expose wheel hubs and non-spinning wheel bearing frames. Inboard shaft ports are owned by the driveline: enable <code>useFrontInboardBrakes</code> or <code>useRearInboardBrakes</code> and connect brakes to <code>inboardBrake_1</code> through <code>inboardBrake_4</code>. Halfshafts always terminate at the standard chassis wheel hubs. The brake rotor taps the inboard side of shaft compliance; its caliper bearing frame belongs to the sprung mount. Outboard brakes attach at the wheel hub and upright bearing frame. The vehicle template selects these connections from brake placement.</p>
<p>An undriven front axle with inboard brakes still requires shafts; the rear-drive adapter provides independent front brake shafts with free differential-side ends and shared <code>pDriveline.frontBrakeHalfshaft</code> calibration. Shaft presence is therefore distinct from drive torque topology. The single differential-input brake remains upstream of the rear final drive.</p>
<p>KnC and four-post fixtures own their static shaft-reaction paths inside <code>RigAxleBrake</code>; suspension models have no brake-location switch. Brake location changes link loads and jacking reactions. The ideal carrier-motion model preserves torque and power transfer but does not predict CV-joint plunge forces, joint friction, or shaft bending.</p>
<p>The vehicle uses its actual open/locked/LSD differential. The prescribed four-post sweep evaluates static reactions using ideal open-differential torque sharing; its input-brake torque is divided by <code>frontBrakeInputRatio</code> or <code>rearBrakeInputRatio</code>. The vehicle's <code>pVehicle.driveline.inputRatio</code> is the reduction after the separate transmission. The shaft adapter includes carrier motion for dynamic power balance but does not model CV-joint geometry or shaft bending. Do not add a second mounting torque: the adapter relocates the reaction already implicit in the lumped contact-patch moment path.</p>
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
<h4>Powertrain supports and rotor effects</h4>
<p>VehicleSim and VehicleFMI always mount the powertrain to chassis.chassisFrame
through the VI transmission bearing path and one rigid powertrain mount.
Motor stator, gear housing and brake caliper reactions are active independently
of rotor fidelity. Outboard brake bearings follow the upright; inboard and
differential-input brake bearings follow the driveline housing.</p>
<p><code>PartialElectricVehicle.includeRotorEffects</code> selects Rotor1D
instead of scalar spin inertia, adding gyroscopic and carrier-acceleration
corrections. It does not change connectors, mounting, or brake placement.
The default is false; this neglects rotor/carrier inertial coupling while
retaining physical support torques. Tire slip always uses absolute wheel spin.
The vehicle keeps world.driveTrainMechanics3D=true for Mounting1D support
conversion in both cases. Standalone 1D components can still omit their VI
bearing connectors.</p>
<p>Each VehicleDefn mass record must contain the complete locked-rotor mass,
center of mass, and inertia tensor. Rotor1D adds relative-spin corrections;
do not subtract spin inertia from the body tensor or add another rotor body.
The ideal shaft redirects torque axes with its support reaction lumped at the
sprung frame. It conserves spatial moment and physical rotor power but does
not resolve CV-joint forces, shaft bending, or detailed joint geometry.</p>
<p>The four-post and KnC fixtures retain their prescribed-wheel, static reaction model.
They enable world.driveTrainMechanics3D=true and expose explicit caliper supports.
Outboard calipers attach to the unsprung upright frames; inboard and differential
calipers attach to the sprung frame. Shaft adapters only redirect torque axes;
they do not add a duplicate caliper reaction.
Its differential brake split is an ideal rig loading condition, not a dynamic
limited-slip differential prediction.</p>
<h4>Parameter conventions</h4><p>Model dialogs place calibration records first, followed by architecture and connector choices, operating or experiment settings, initialization, and animation or advanced controls. Existing specialized groups retain physical meaning, such as spring and damper curves. Each locally declared parameter has a description; SI types define units, and descriptions identify coordinate frames, table columns and special conventions where needed. A record field order is preserved to avoid changing positional record constructors. Final and protected derived quantities are implementation details, not additional calibration inputs. Inherited MSL and VehicleInterfaces parameters retain their upstream contracts.</p><p>Calibrate through the subsystem record, for example <code>pMotor.reverseLaunch</code>, rather than introducing duplicate scalar aliases. Record values are initialization-time configuration, not necessarily changeable during integration. Architecture switches and model redeclarations are compile-time choices.</p>
</html>"));
end ModifyingVehicleConstruction;
