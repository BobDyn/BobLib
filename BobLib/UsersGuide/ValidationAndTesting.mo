within BobLib.UsersGuide;
class ValidationAndTesting

  "How to check BobLib changes before trusting results"
  extends Modelica.Icons.Information;

  annotation(
    preferredView = "info",
    Documentation(info = "<html>
<h4>Validation Philosophy</h4>
<p>
The included models are regression-tested baselines, not validated replicas of
any specific vehicle. Before using outputs for design decisions, validate the
relevant records and subsystems against measured data. FSAE teams should treat
BobLib as a strong starting architecture and a testable modeling layer, not as
an automatic substitute for vehicle identification.
</p>
<h4>Local Test Targets</h4>
<p>
The repository makefile runs pytest-based checks. Common targets are:
</p>
<table border='1' cellspacing='0' cellpadding='4'>
<tr><th>Command</th><th>Purpose</th></tr>
<tr><td><code>make ci</code></td><td>Run Python lint and the full local test gate.</td></tr>
<tr><td><code>make test</code></td><td>Run pure Python tests and all Modelica checks.</td></tr>
<tr><td><code>make modelica-lint</code></td><td>Check Modelica formatting for production and test libraries.</td></tr>
<tr><td><code>make modelica-format</code></td><td>Rewrite Modelica formatting in the configured paths.</td></tr>
<tr><td><code>make modelica-translation</code></td><td>Translate standards and BobLibTest fixtures through pytest.</td></tr>
<tr><td><code>make modelica-initialization</code></td><td>Initialize BobLibTest fixtures and compare initialization baselines.</td></tr>
<tr><td><code>make modelica-physics</code></td><td>Simulate physical validation baselines with hardware-tolerant runtime budgets.</td></tr>
<tr><td><code>make modelica-regression</code></td><td>Run signal-level regressions and smoke checks.</td></tr>
</table>
<p>
Install Modelica dependencies with <code>make modelica-deps</code> when needed.
The current checks target Modelica Standard Library 4.1.0 and
VehicleInterfaces 2.0.2.
</p>
<h4>Test Library Layout</h4>
<p>
Modelica component and regression fixtures live in the sibling package
<code>Tests/BobLibTest</code>, not in the production <code>BobLib</code>
package. Python pytest files live directly under <code>Tests</code>. This keeps
production models clean while still making every regression independently
traceable from pytest output.
</p>
<p>
Representative test areas include:
</p>
<ul>
<li><code>Tests/test_boblib_modelica.py</code>: focused translation and structural checks.</li>
<li><code>Tests/test_modelica_translation.py</code>: translation fixtures.</li>
<li><code>Tests/test_modelica_initialization.py</code>: initialization baseline checks.</li>
<li><code>Tests/test_modelica_physics_validation.py</code>: physical validation baselines.</li>
<li><code>Tests/test_modelica_regression.py</code>: simulation regressions and smoke checks.</li>
<li><code>Tests/test_modelica_linter.py</code>: Modelica formatting checks.</li>
</ul>
<h4>What FSAE Teams Should Validate</h4>
<ul>
<li>Mass, CG, inertia, wheelbase, track, and corner weights.</li>
<li>Suspension pickup points, motion ratios, spring rates, bar rates, damper curves, steering ratio, toe, camber, caster, and ride heights.</li>
<li>Tire radius, load sensitivity, slip behavior, relaxation behavior, and normal-load traces.</li>
<li>Aero map sign conventions, reference density, reference speed, center of pressure, and ride-height sensitivity.</li>
<li>Battery voltage/current limits, inverter limits, motor torque and power limits, final-drive ratio, differential behavior, and halfshaft stiffness and damping.</li>
<li>Brake torque capacity, brake bias, pedal mapping, regen blend, and mechanical brake response.</li>
<li>Controller gains, target speed behavior, steering command behavior, and actuator limits.</li>
<li>Track-test signals such as speed, yaw rate, lateral acceleration, handwheel angle, handwheel torque, wheel speeds, and tire normal loads.</li>
</ul>
<h4>Adding or Updating Baselines</h4>
<p>
Add the smallest test that catches the behavior you changed. A new lumped
model usually deserves a component fixture under <code>Tests/BobLibTest</code>

and a pytest translation or simulation check. A core physics change usually
also deserves initialization and physical-baseline coverage.
</p>
<p>
Only update baselines when the physical change is intentional and understood.
Keep the old and new signal behavior available during review when practical,
especially for suspension, tire, aero, differential, and VCU changes.
</p>
<h4>Runtime Checks</h4>
<p>
Runtime budgets vary by hardware, so BobLib checks use hardware-tolerant
scaling. Treat a runtime regression as a modeling clue. Common causes include
stiff contact mechanics, poorly balanced initialization, unnecessary animation,
over-constrained state selection, excessive event generation, or discontinuous
controller logic. For batch runs, use <code>headless = true</code> unless
animation geometry is being inspected.
</p>
<h4>BobLib and BobSim</h4>
<p>
Use BobLib tests to protect Modelica package health. Use BobSim for repeatable
study execution, signal extraction, plots, reports, envelope maps, and
sensitivity studies. A production workflow generally needs both: BobLib for the
model layer and BobSim for the study/report layer.

</p>
</html>"));
end ValidationAndTesting;
