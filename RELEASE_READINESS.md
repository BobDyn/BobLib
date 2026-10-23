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
speed, DASSL, 1e-6 tolerance and local homotopy. No initialization baseline was
regenerated. The compiler reported initialization without homotopy in all six runs.

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
