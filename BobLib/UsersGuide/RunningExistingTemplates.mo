within BobLib.UsersGuide;
class RunningExistingTemplates

  "How to run the included standard templates"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Choose an Entrypoint</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Entrypoint</th><th>Use when</th><th>Architecture</th></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.VehicleSim'>VehicleSim</a></td><td>You want the standard autonomous full-vehicle maneuver model.</td><td>EV battery, inverter, motor, fixed-ratio transmission, rear final-drive differential, chassis, brakes, VCU, driver environment, road, atmosphere, aero, and world.</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.VehicleFMI'>VehicleFMI</a></td><td>You want an FMI, SIL, or DIL boundary driven by driver inputs.</td><td>Same EV plant style, but the public interface is only steering wheel, accelerator pedal, and brake pedal.</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.FourPostSim'>FourPostSim</a></td><td>You want suspension and tire load-path checks without the full powertrain stack.</td><td>Four-post fixture around the chosen front and rear suspension architecture.</td></tr>
</table>
<h4>Full-Vehicle Templates</h4>
<p>
The main full-vehicle template is
<a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.BaseVehicleSim'>BaseVehicleSim</a>.
It owns the plant wiring, physical connections, shared
<a href='modelica://VehicleInterfaces.Interfaces.ControlBus'>ControlBus</a>,
road, atmosphere, aero model, world, standard outputs, and maneuver termination
monitors. It does not own the detailed control law; standard maneuver
generation and longitudinal control live in
<a href='modelica://BobLib.Controllers.StandardVCU'>StandardVCU</a>.
</p>
<p>
Concrete variants live under
<a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle'>Templates.Vehicle</a>.
Their names encode the major plant construction. This keeps the standard
templates aligned with the way FSAE teams usually compare vehicle
architectures: one EV powertrain stack and several front/rear suspension
combinations.
</p>
<p>
The public
<a href='modelica://BobLib.Experiments.Standards.VehicleSim'>VehicleSim</a>
extends the default concrete variant so users have a stable entry point while
templates remain available for architecture comparisons.
</p>
<h4>Architecture Tokens</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Token</th><th>Meaning</th></tr>
<tr><td><code>EVBatInvMotDiff</code></td><td>Electric FSAE-style powertrain: battery pack, inverter, electric motor, fixed-ratio transmission, and rear final-drive differential.</td></tr>
<tr><td><code>DWDirect</code></td><td>Double-wishbone axle with a direct spring/damper linkage.</td></tr>
<tr><td><code>DWBC</code></td><td>Double-wishbone axle with bellcrank spring/damper actuation.</td></tr>
<tr><td><code>DWBCStabar</code></td><td>Double-wishbone bellcrank axle with stabilizer bar.</td></tr>
</table>
<p>
Vehicle template names use
<code>VehicleSim_EVBatInvMotDiff_FRONT_REAR</code>. The first suspension token
is the front axle, and the second suspension token is the rear axle.
</p>
<h4>Available Full-Vehicle Variants</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Variant</th><th>Front</th><th>Rear</th></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC'>VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBC</a></td><td>Bellcrank plus stabilizer bar</td><td>Bellcrank</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar'>VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar</a></td><td>Bellcrank plus stabilizer bar</td><td>Bellcrank plus stabilizer bar</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect'>VehicleSim_EVBatInvMotDiff_DWBCStabar_DWDirect</a></td><td>Bellcrank plus stabilizer bar</td><td>Direct</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBC_DWBC'>VehicleSim_EVBatInvMotDiff_DWBC_DWBC</a></td><td>Bellcrank</td><td>Bellcrank</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBC_DWBCStabar'>VehicleSim_EVBatInvMotDiff_DWBC_DWBCStabar</a></td><td>Bellcrank</td><td>Bellcrank plus stabilizer bar</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBC_DWDirect'>VehicleSim_EVBatInvMotDiff_DWBC_DWDirect</a></td><td>Bellcrank</td><td>Direct</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWDirect_DWBC'>VehicleSim_EVBatInvMotDiff_DWDirect_DWBC</a></td><td>Direct</td><td>Bellcrank</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWDirect_DWBCStabar'>VehicleSim_EVBatInvMotDiff_DWDirect_DWBCStabar</a></td><td>Direct</td><td>Bellcrank plus stabilizer bar</td></tr>
<tr><td><a href='modelica://BobLib.Experiments.Standards.Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWDirect_DWDirect'>VehicleSim_EVBatInvMotDiff_DWDirect_DWDirect</a></td><td>Direct</td><td>Direct</td></tr>
</table>
<h4>Common Parameters</h4>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Parameter</th><th>Meaning</th></tr>
<tr><td><code>pVehicle</code></td><td>The complete vehicle record. It aggregates subsystem-owned records such as battery, inverter, motor, driveline, aero, chassis, and VCU data.</td></tr>
<tr><td><code>initialVel</code></td><td>Initial vehicle speed and tire spin initialization source.</td></tr>
<tr><td><code>headless</code></td><td>Set to <code>true</code> for batch runs without MultiBody animation geometry.</td></tr>
<tr><td><code>vcu.useMode</code></td><td>Standard maneuver selection in <code>StandardVCU</code>.</td></tr>
<tr><td><code>vcu.targetVel</code></td><td>Longitudinal speed target for the VCU speed controller.</td></tr>
<tr><td><code>vcu.targetAy</code></td><td>Lateral acceleration target or direction, depending on maneuver mode.</td></tr>
</table>
<h4>Maneuver Modes</h4>
<p>
<a href='modelica://BobLib.Controllers.StandardVCU'>StandardVCU</a> defines
four standard modes:
</p>
<ul>
<li><code>MODE_OPEN_LOOP_RAMP = 0</code>: ramp steer until normal-load or linearity criteria stop the ramp, then terminate after quasi-steady response settles.</li>
<li><code>MODE_OPEN_LOOP_SINE = 1</code>: sinusoidal steering command.</li>
<li><code>MODE_STEP_STEER = 2</code>: step or short ramp steering command.</li>
<li><code>MODE_STEADY_STATE_AY = 3</code>: closed-loop steering to a desired steady-state lateral acceleration while the inherited VCU controls speed.</li>
</ul>
<p>
Mode 3 is the preferred choice when you want the vehicle to drive itself toward
a requested steady-state cornering condition. The open-loop ramp mode is useful
for extracting smooth nonlinearity and limit behavior from a repeatable steer
ramp.
</p>
<h4>FMI, SIL, and DIL Boundary</h4>
<p>
<a href='modelica://BobLib.Experiments.Standards.Templates.FMI.BaseVehicleFMI'>BaseVehicleFMI</a>
is the minimal driver-input boundary. It exposes only:
</p>
<ul>
<li><code>steeringAngleCommand</code></li>
<li><code>acceleratorPedalCommand</code></li>
<li><code>brakePedalCommand</code></li>
</ul>
<p>
Those signals enter
<a href='modelica://BobLib.DriverEnvironments.Internal.Driver'>Driver</a>,
which publishes driver intent onto <code>controlBus.driverBus</code>. The
preconfigured
<a href='modelica://BobLib.Controllers.VCU'>VCU</a> subscribes to the driver
and plant buses, then publishes actuator requests on the downstream control
buses. EV-specific internal signals are not public FMI inputs. Optional
drive-by-wire adapters such as
<a href='modelica://BobLib.DriverEnvironments.EVDriveByWire'>EVDriveByWire</a>
belong outside this minimal base interface.
</p>
<h4>Four-Post Templates</h4>
<p>
<a href='modelica://BobLib.Experiments.Standards.Templates.FourPost.BaseFourPostSim'>BaseFourPostSim</a>
is the suspension fixture template. Concrete variants live under
<a href='modelica://BobLib.Experiments.Standards.Templates.FourPost'>Templates.FourPost</a>
and vary the front and rear suspension construction. Use these models for
spring, bar, contact-patch force, ride-height, and load-transfer checks before
trusting a full vehicle maneuver result.
</p>
<p>
The four-post package uses the same front/rear suspension tokens as the
full-vehicle package, with names such as
<a href='modelica://BobLib.Experiments.Standards.Templates.FourPost.FourPostSim_DWBCStabar_DWBCStabar'>FourPostSim_DWBCStabar_DWBCStabar</a>
and
<a href='modelica://BobLib.Experiments.Standards.Templates.FourPost.FourPostSim_DWDirect_DWDirect'>FourPostSim_DWDirect_DWDirect</a>.
The public
<a href='modelica://BobLib.Experiments.Standards.FourPostSim'>FourPostSim</a>
is the stable default entry point.
</p>
<h4>Running and Checking</h4>
<p>
In OMEdit, open <code>BobLib/package.mo</code>, load Modelica 4.1.0 and
VehicleInterfaces 2.0.2, then check or simulate the desired entry point. For
batch health checks, use the repository make targets described in
<a href='modelica://BobLib.UsersGuide.ValidationAndTesting'>Validation and Testing</a>.
</p>
<p>
For fast batch simulation, set <code>headless = true</code>. Keep animation on
when inspecting geometry, steering direction, wheel travel, and force paths.
</p>
</html>"));
end RunningExistingTemplates;
