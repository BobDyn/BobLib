within BobLib.UsersGuide;
class ModifyingCorePhysics

  "How to modify suspension, tires, contact mechanics, and other core physics"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Core Physics Boundary</h4>
<p>
Core physics models are the detailed MultiBody and tire models beneath the
public VehicleInterfaces adapters. The public chassis boundary is
<a href='modelica://BobLib.Chassis.ChassisBase'>ChassisBase</a>. It extends
the VehicleInterfaces two-axle chassis contract, owns the chassis
<code>FreeMotion</code> state holder, closes contact-patch frames to ground,
and publishes chassis-owned measurements on <code>chassisBus</code>.
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
Double-wishbone axles derive from
<a href='modelica://BobLib.Chassis.Suspension.AxleDWBase'>AxleDWBase</a>.
Concrete front and rear axles include
<a href='modelica://BobLib.Chassis.Suspension.FrAxleDW_BC'>FrAxleDW_BC</a>,
<a href='modelica://BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar'>FrAxleDW_BC_Stabar</a>,
<a href='modelica://BobLib.Chassis.Suspension.FrAxleDW_Direct'>FrAxleDW_Direct</a>,
<a href='modelica://BobLib.Chassis.Suspension.RrAxleDW_BC'>RrAxleDW_BC</a>,
<a href='modelica://BobLib.Chassis.Suspension.RrAxleDW_BC_Stabar'>RrAxleDW_BC_Stabar</a>,
and
<a href='modelica://BobLib.Chassis.Suspension.RrAxleDW_Direct'>RrAxleDW_Direct</a>.
</p>
<p>
The lower-level kinematic loop is
<a href='modelica://BobLib.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoop'>WishboneUprightLoop</a>.
Linkage models such as
<a href='modelica://BobLib.Chassis.Suspension.Linkages.Bellcrank2'>Bellcrank2</a>,
<a href='modelica://BobLib.Chassis.Suspension.Linkages.Bellcrank3'>Bellcrank3</a>,
<a href='modelica://BobLib.Chassis.Suspension.Linkages.ShockLinkage'>ShockLinkage</a>,
<a href='modelica://BobLib.Chassis.Suspension.Linkages.ForceOnlyRod'>ForceOnlyRod</a>,
<a href='modelica://BobLib.Chassis.Suspension.Templates.Stabar.Stabar'>Stabar</a>,
and
<a href='modelica://BobLib.Chassis.Suspension.Templates.SteeringRack.RackAndPinion'>RackAndPinion</a>
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
<a href='modelica://BobLib.Chassis.Suspension.Tires.TirePhysics'>TirePhysics</a>,
with the shared partial wheel at
<a href='modelica://BobLib.Chassis.Suspension.Tires.TirePhysics.Templates.PartialWheel'>PartialWheel</a>.
Slip models live under
<a href='modelica://BobLib.Chassis.Suspension.Tires.MF52.SlipModel'>MF52.SlipModel</a>,
including kinematic, transient, and no-slip variants.
</p>
<h4>State Selection</h4>
<p>
Detailed MultiBody models can become singular or slow when too many equivalent
coordinates compete to become states. The goal is not to force every variable;
the goal is to identify the smallest physically independent state set and make
initialization obvious.
</p>
<ul>
<li>Use one clear state holder for the vehicle body. <code>ChassisBase</code> currently uses <code>cgFreeMotion</code> with fixed initial position, orientation, angular velocity, and initial longitudinal velocity.</li>
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
