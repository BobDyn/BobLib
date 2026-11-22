# Release-readiness scope

This stack prepares the Modelica library for review; it does not certify a release.
The foundation is the preserved pre-SS release candidate of 2026-10-20. Native/FMU export tooling and calibration tests are included. VehicleSimSS,
SS initialization hooks and GGV prototypes are excluded. The FMU exporter already merged in PR #30 is retained. Later GGV-related plant changes remain separate. The
existing dynamic maneuver controller and VehicleFMI Modelica boundary remain.

Layers separate package styling, calibration contracts, typed buses, mechanics,
brakes, suspension components, wheel compliance, aero, differential components,
drivers and electrical drive. The coupled vehicle API migration follows, then
fixture coverage, domain regression checks and CI/migration documentation. Two final tooling layers add configurable FMU export, then native export and
cross-runtime calibration verification. They each contain corresponding tests.

## Fresh isolated checks

- Six public entry points passed OpenModelica checkModel with balanced equations:
  VehicleSim, VehicleFMI, FourPostSim, KnCSteerable, KnCLocked and ManeuverFMI.
- 1,067 tests collected without dependencies on the excluded tooling.
- 613 focused import, routing, diagram, coverage, parameter-documentation,
  architecture and formatting-helper tests passed.
- The mounted-transmission regression passed with unique simulation output prefixes,
  fixing the previous filename/GUID collision.
- Full-tree Modelica formatting passed after an annotation-only MassRecord fix.

This is not a fresh full simulation-suite pass. The historical snapshot reported
1,302 passes and one fixture failure, including checks outside this stack; that
result cannot certify these extracted commits.

## Remaining before release

Run the full gate on final merged sources, justify baseline changes case by case,
complete OMEdit review of every model and relevant conditional variant, and validate
numerical behavior across supported architectures. Native/FMU artifact acceptance must use the final exact source identity; a single
architecture validation is not acceptance of the complete catalog.

The merged FMU-export PR #30 has been incorporated. Reconcile the unmerged
damper-table work against the new package and record paths; compatibility with
unmerged PRs is not established by these drafts.

## Vehicle behavior comparison for PR #48

A fresh comparison used pre-stack `a3932b3` (package styling only) and `0729995`
with OpenModelica 1.27.0-cmake, MSL 4.1.0 and VehicleInterfaces 2.0.2. Both use
the shipped bellcrank/stabilizer vehicle, its saved starting pose, 15 m/s initial
speed, DASSL, 1e-6 tolerance and local homotopy. The vehicle starting pose and
physical calibration were preserved. The compiler reported initialization without homotopy in all six runs.

| Measurement | Before | After |
|---|---:|---:|
| Default ramp QSS exit time (s) | 6.138183 | 6.096960 |
| Ramp final lateral acceleration (m/s²) | 16.914860 | 16.614094 |
| Ramp final yaw rate (rad/s) | 1.136999 | 1.116581 |
| Ramp minimum tire load throughout run (N) | 182.828490 | 183.542343 |
| Straight braking speed at 2 s, target 10 m/s (m/s) | 10.439863 | 10.438521 |
| Straight acceleration speed at 2 s, target 20 m/s (m/s) | 19.487432 | 19.488775 |

All runs ended normally with finite recorded signals and positive tire loads.
The new ramp reports `terminationStatus=1`; the two fixed-horizon straight runs
report 0. The new default ramp reaches about 1.8% lower lateral acceleration and
yaw rate. Straight-line final speeds differ by about 0.00134 m/s. These are
aggregate differences across the stack, including the clutch differential,
shaft inertia, reaction paths and controller split. They do not isolate the
contribution of each change, establish physical equivalence, or certify launch
from rest, wheel lock, every architecture or the full operating envelope.
The rear-inboard default describes the supplied reference configuration; callers
modeling outboard hardware must select `rearBrakePlacement=Outboard` explicitly.

Machine-readable results, source identities and the remaining signals are in
`Tests/review_results/vehicle_stack_comparison.json`. Reproduce with:

```sh
python Tests/compare_vehicle_stack.py --before /path/to/a3932b3-worktree \
  --after /path/to/0729995-worktree --output /tmp/boblib-comparison
```

The script retains compiler scripts/logs and CSV results. It compares the default
ramp and a wrapper with a literal speed target and delayed steering; the latter
allows a checked runtime target override for the acceleration/braking cases.
It rejects unknown or ignored overrides instead of silently comparing defaults.

Seven behavioral tests in `Tests/test_vehicle_termination.py` passed on a fresh
full-vehicle build: running, settled, tire limit, sideslip limit, both settling
timeouts and invalid monitor configuration. Envelope/timeout exits retain the
result and expose a distinct status; invalid configuration still fails.

The initialization snapshot for `VehicleSimAnimationOn` was subsequently updated
for the new `terminationStatus` output. A fresh before/after comparison against
`0b7ab76` verified the old snapshot, found exactly one added numeric variable
(`terminationStatus=0`), no removed variables, and no existing initialized value
outside the unchanged 1e-9 relative / 1e-8 absolute tolerances. Only that one
snapshot row changed (2,683 to 2,684 numeric/finite variables); no physical
initialization values or tolerances were adjusted.

## VehicleSim steering and settling follow-up

The steady-state driver's sampled PI demand now drives a continuous two-stage,
rate-limited handwheel state with continuous velocity. `steadyStateMaxHandwheelRate` defaults to 0.5 rad/s;
it is a test-actuator setting and must be matched to the intended experiment.
The state remains explicit for full-vehicle initialization, without controller
sample-time resets. Step-steer settling starts after the commanded transition,
checks speed and sampled speed/lateral/yaw/roll/sideslip rates, and holds for
`stepSteerHoldDuration` (default 0.5 s). Step and closed-loop holds also monitor
continuous signal excursions, including speed, so sample-aligned oscillations cannot pass solely
because their finite differences vanish. These protocol changes require renewed
vehicle correlation; fixture checks do not certify runtime or real-vehicle accuracy.

## Opt-in steady-state continuation

`enableSteadyStateSequence` enables a fixed-capacity list of up to 64 Ay targets.
The scalar `driver.targetAy` contract remains the default. Each accepted hold
emits `BOBLIB_STEADY_POINT` with its index, physical time and target, independently
of CSV event emission. The next smooth target transition starts from the prior
command, preserving the mechanical, steering and PI states. Settling timeout is
relative to each target's transition; tire-load and sideslip exits remain active.
A limit exit does not accept the unfinished point. BobSim supplies a zero-Ay
preconditioning point before the requested sweep. Independent starts, reverse
sweeps and full-vehicle runtime comparisons are required before making this mode
the default; continuation can expose history-dependent branches.

The VehicleSim settling diagnostics add one initialized numeric variable to
`VehicleSimAnimationOn` (2684 to 2685). Its initialization snapshot was regenerated
with OpenModelica 1.27.1 after inspecting the zero-valued `steadyStateSpeedRate` diagnostic. Removing
that output reproduces the previous variable-name hash. The
sum, absolute sum and maximum of initialized values are unchanged, and no
nonfinite values were introduced. Only this fixture's snapshot changed; no
comparison tolerances were relaxed.
