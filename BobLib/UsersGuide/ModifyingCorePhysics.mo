within BobLib.UsersGuide;
class ModifyingCorePhysics

  "How to modify suspension, tires, contact mechanics, and other core physics"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Averaged Electrical Contract</h4>
<p>
<code>PowerElectronics.InverterDC</code> supplies the motor-side voltage and
publishes signed <code>electricMotorControlBus.availablePower</code> in watts.
<code>ElectricDrives.Motor</code> consumes that allocation with its own current
law; positive power is motoring and negative power is regeneration. Connect
both electrical pins and the shared control bus. The motor uses the existing
<code>BobLib.ElectricDrives.Interfaces.Base</code> contract.
</p>
<p>
A replacement averaged motor must subscribe to the allocation. A replacement
inverter must publish it. Matching electrical connectors alone does not ensure
that two arbitrary control or energy-conversion laws are compatible. This
averaged interface does not model switching waveforms. The motor's
<code>V_eps</code> regularizes current near zero voltage; normal operation is
above the inverter's undervoltage threshold.
</p>
<h4>Core Physics Boundary</h4>
<p>
Core physics models are the detailed MultiBody and tire models beneath the
public VehicleInterfaces adapters. The public chassis boundary is
<a href='modelica://BobLib.Chassis.BaseClasses.PartialChassis'>PartialChassis</a>. It extends
the VehicleInterfaces two-axle chassis contract, owns the chassis
<code>FreeMotion</code> state holder, closes contact-patch frames to ground,
and publishes chassis-owned measurements on <code>chassisBus</code>.
</p>
<p>
The adapter inherits its front axle, sprung body, and rear axle from
<code>Chassis.BaseClasses.PartialChassisAssembly</code>, keeping all three visible in
the diagram. Axles publish their own measurements using the VehicleInterfaces
<code>ChassisBus</code> type on <code>frAxleDW.axleBus</code> and
<code>rrAxleDW.axleBus</code>. Each supplies

<code>leftNormalLoad</code>, <code>rightNormalLoad</code> (N),
<code>leftSpringLength</code>, and <code>rightSpringLength</code> (m).
The signal adapter maps axle loads to the existing flat chassis bus fields for
vehicle subscribers. Body speed, acceleration, and aero-reference ride heights
remain chassis-owned. Connect a typed <code>VehicleInterfaces.Interfaces.ChassisBus</code>
tap to an axle bus when subscribing to axle-specific measurements.
</p>
<p>
The detailed suspension, tire, steering, body, and contact geometry is nested
under <a href='modelica://BobLib.Chassis'>Chassis</a> and
<a href='modelica://BobLib.Utilities.Mechanics.MultiBody'>Utilities.Mechanics.MultiBody</a>.
Reusable frame fixtures, contact mechanics, and rig actuators belong in
<code>Utilities.Mechanics.MultiBody</code>, not deep inside a tire package.
</p>
<h4>Suspension Stack</h4>
<p>
Choose a concrete axle at the root of <code>Chassis.Suspension</code>.
<code>Interfaces</code> defines the architecture-neutral connection contract;
<code>BaseClasses</code> contains partial implementations intended for extension;
<code>Components</code> contains reusable mechanisms. These are different reuse
boundaries, not successive inheritance layers. A concrete axle extends its
shared base once, and that base already extends the axle interface.
</p>
<p>
Double-wishbone axles derive from
<a href='modelica://BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle'>PartialDoubleWishboneAxle</a>.
The same concrete axle models can be selected independently at the front and rear:
<a href='modelica://BobLib.Chassis.Suspension.DoubleWishboneBellcrank'>DoubleWishboneBellcrank</a>,
<a href='modelica://BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar'>DoubleWishboneBellcrankStabar</a>,
and
<a href='modelica://BobLib.Chassis.Suspension.DoubleWishboneDirect'>DoubleWishboneDirect</a>.
Vehicle records supply the position-specific geometry and calibration; the chassis
or test rig supplies the steering input or restraint. The wishbone base implements
<a href='modelica://BobLib.Chassis.Suspension.Interfaces.AxleBase'>Interfaces.AxleBase</a>,
which contains only shared mechanical connections. Chassis configuration and
initialization still use wishbone-specific data; other architectures must not
inherit wishbone geometry merely to satisfy that configuration.
</p>
<p>
The lower-level kinematic loop is
<a href='modelica://BobLib.Chassis.Suspension.Components.WishboneUprightLoop'>WishboneUprightLoop</a>.
Linkage models such as
<a href='modelica://BobLib.Chassis.Suspension.Components.Bellcrank2'>Bellcrank2</a>,
<a href='modelica://BobLib.Chassis.Suspension.Components.Bellcrank3'>Bellcrank3</a>,
<a href='modelica://BobLib.Chassis.Suspension.Components.ShockLinkage'>ShockLinkage</a>,
<a href='modelica://BobLib.Chassis.Suspension.Components.ForceOnlyRod'>ForceOnlyRod</a>,
<a href='modelica://BobLib.Chassis.Suspension.Components.Stabar'>Stabar</a>,
and
<a href='modelica://BobLib.Chassis.Suspension.Components.RackAndPinion'>RackAndPinion</a>
provide reusable suspension mechanisms.
</p>
<p>
Suspension axle and tire models should expose and connect raw contact-patch
frames. Chassis-level contact mechanics then close those frames to ground using
helpers such as
<a href='modelica://BobLib.Utilities.Mechanics.MultiBody.ContactMechanics.GroundPhysics'>GroundPhysics</a>.
</p>
<h4>Tire Stack</h4>
<p>
Tires live under
<a href='modelica://BobLib.Chassis.Suspension.Tires'>Chassis.Suspension.Tires</a>
because suspension axles own wheel centers, tire load paths, and raw
contact-patch frames. The public tire adapter is
<a href='modelica://BobLib.Chassis.Suspension.Tires.BaseTire'>BaseTire</a>.
<a href='modelica://BobLib.Chassis.Suspension.Tires.MF52Tire'>MF52Tire</a>
combines wheel physics and Magic Formula 5.2 evaluation models.
</p>
<p>
Wheel physics variants live under
<a href='modelica://BobLib.Chassis.Suspension.Tires.Wheels'>Wheels</a>,
with the shared partial wheel at
<a href='modelica://BobLib.Chassis.Suspension.Tires.Wheels.PartialWheel'>PartialWheel</a>.
Slip models live under
<a href='modelica://BobLib.Chassis.Suspension.Tires.MF52.SlipModel'>MF52.SlipModel</a>,
including kinematic, transient, and no-slip variants.
</p>
<h4>Replacement Contracts</h4>
<p>
A <code>replaceable</code> declaration chooses the default implementation;
<code>constrainedby</code> defines the public contract that replacements must satisfy.
Use the existing VehicleInterfaces contract when it contains all the members the
parent uses. BobLib extends that contract where necessary: its chassis exposes
measurements and inboard brake flanges. Atmosphere replacements use the VI
function contract directly; they do not require a BobLib atmosphere bus.

Power electronics has a small pin-and-bus interface because VehicleInterfaces
2.0.2 does not provide that subsystem contract.
</p>
<p>
Wheel replacements are constrained by <code>Wheels.PartialWheel</code>, slip
replacements by <code>SlipModel.BaseSlipModel</code>, and frame replacements by
<code>Chassis.Body.FrameBase</code>. Wishbone configuration slots retain
<code>PartialDoubleWishboneAxle</code> because their configuration uses wishbone
geometry. An architecture-neutral connection contract alone cannot promise that geometry.
</p>
<p>
Put shared calibration and assembly requirements on the constraining type so
those modifiers survive redeclaration. Keep implementation-specific tuning on
the default declaration. For example, <code>BaseTire</code> binds
<code>partialWheelParams=pPartialWheel</code> on its wheel constraint; a new wheel
keeps that geometry without repeating the binding. Use <code>redeclare replaceable</code>
when a derived model should leave the selection open to further replacement.
A compatible replacement must also provide balanced equations and valid initialization;
matching the contract alone does not establish physical correctness.
</p>
<p>
See <a href='https://mbe.modelica.university/components/architectures/replaceable/'>
Modelica by Example: Replaceable</a> for the distinction between default and
constraining types and the lifetime of their modifiers.
</p>
<h4>Compliant Laws</h4>
<p>
Each concrete suspension axle exposes <code>SpringModel</code> and
<code>DamperModel</code>. Selecting either once applies it to both sides.
The defaults use the axle record's tabulated curves. A replacement supplies its
own calibration; for example,
<code>redeclare model SpringModel = Modelica.Mechanics.Translational.Components.Spring(c=30000, s_rel0=0.2)</code>.
The standard MSL two-flange contract allows a different force law without
requiring it to understand BobLib tables. Standalone <code>ShockLinkage</code>
exposes the corresponding <code>TabularSpring</code> and <code>TabularDamper</code>
component slots for redeclaration.
</p>
<p>
<code>RearFinalDriveDifferential.HalfshaftModel</code> similarly configures both
halfshaft compliance laws. Each <code>Drivelines.Internal.Halfshaft</code>
contains the rotor inertia, optional spatial mount, and compliance element.
Both receive the same <code>pDriveline.halfshaft</code> record.
The compliance element is now accessed as <code>leftHalfshaft.compliance</code>
or <code>rightHalfshaft.compliance</code>; the model-level redeclaration remains unchanged. The torsional
frame, stabilizer bar, compliant rod and vertically compliant wheels expose
their force elements through the corresponding MSL compliant interfaces.
</p>
<p>
Mixed-unit tables use <code>Real[:,2]</code>: spring columns are displacement
[m] and force [N], while damper columns are velocity [m/s] and force [N]. A
stiffness or damping type on the entire matrix would mislabel both columns.
</p>
<h4>State Selection</h4>
<p>
Detailed MultiBody models can become singular or slow when too many equivalent
coordinates compete to become states. The goal is not to force every variable;
the goal is to identify the smallest physically independent state set and make
initialization obvious.
</p>
<ul>
<li>Use one clear state holder for the vehicle body. <code>PartialChassis</code> currently uses <code>cgFreeMotion</code> with fixed initial position, orientation, angular velocity, and initial longitudinal velocity.</li>
<li>Give tire spin states explicit starts from <code>initialLongitudinalVelocity / wheelRadius</code>.</li>
<li>Do not assign <code>StateSelect.always</code> inside every joint of a closed kinematic loop. Prefer independent coordinates and let dependent loop coordinates remain algebraic.</li>
<li>Use <code>StateSelect.prefer</code> or <code>StateSelect.always</code> only when it resolves an actual ambiguity and after checking translation and initialization.</li>
<li>Make fixed starts intentional. A fixed start value is a constraint; an unfixed start value is a hint.</li>
<li>For stiff contact or linkage models, verify that initialization force balance is reasonable before tuning solver flags.</li>
</ul>
<h4>Initialization and Singular Systems</h4>
<p>
If OpenModelica reports a singular linear system at <code>time = 0</code>, look
first for redundant kinematic constraints, missing fixed starts on the intended
state set, impossible initial geometry, or contact mechanics fighting gravity
and spring preload. Solver fallbacks can hide the issue temporarily but usually
hurt FMU and batch performance. Prefer prescribing the physical state set and
consistent starts in the model.
</p>
<p>
For a full vehicle, check tire normal loads, chassis position, tire spin,
roll/pitch/yaw starts, spring deflections, and contact-patch heights. For a
four-post or suspension fixture, check that imposed rig motion, spring/bar
rates, and contact-patch frames produce the expected static loads before
running dynamic maneuvers.
</p>
<h4>Contact Mechanics</h4>
<p>
Contact-patch frames are shared physical boundaries. Tires expose them, axles
route them, and chassis or fixture models close them through reusable
MultiBody utilities. This allows the same tire and suspension models to be used
in full-vehicle, four-post, and component-test contexts.
</p>
<h4>Core Physics Checklist</h4>
<ul>
<li>Keep public VehicleInterfaces adapters stable.</li>
<li>Put reusable frame-based helpers in <code>Utilities.Mechanics.MultiBody</code>.</li>
<li>Put scalar, array, and record calculations in <code>Utilities.Mechanics.Functions</code>.</li>
<li>Keep tire models under <code>Chassis.Suspension.Tires</code>.</li>
<li>Document intentional state selection and fixed starts in code comments or model documentation.</li>
<li>Add component tests under <code>Tests/BobLibTest</code> before relying on a full-vehicle result.</li>
<li>Run translation, initialization, physics, and regression checks after changing states or constraints.</li>
</ul>
</html>"));
end ModifyingCorePhysics;
